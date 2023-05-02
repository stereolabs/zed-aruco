///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/***********************************************************************************************
 ** This sample demonstrates how to reloc a ZED camera using an ArUco marker.                  **
 ** Images are captured with the ZED SDK and cameras poses is then computed from ArUco pattern **
 ** to reset ZED tracking with this known position.                                            **
 ***********************************************************************************************/

#ifdef _DEBUG
#error Please build the project in Release mode
#endif

// Standard includes
#include <stdio.h>
#include <string.h>

// ZED includes
#include <sl/Camera.hpp>
#include <sl/Fusion.hpp>

// OCV includes
#include <opencv2/opencv.hpp>

// Sample includes
#include "aruco.hpp"
#include "GLViewer.hpp"

// Using std and sl namespaces
using namespace std;
using namespace sl;

/**
Global struct to store all needed infos
 */
struct CameraData {
    Camera zed;
    Mat point_cloud;
    Mat im_left;
    cv::Mat im_left_ocv;
    cv::Mat im_left_ocv_rgb;
    cv::Matx33d camera_matrix = cv::Matx33d::eye();
    cv::Matx<float, 4, 1> dist_coeffs = cv::Vec4f::zeros();    
    sl::Transform aruco_transf;

    int id = -1;
    bool is_reloc = false;
    string info;
};

/**
ArUco related data
 */
struct ArucoData {
    aruco::Dictionary dictionary;
    float marker_length;
};

// Global objetcts
bool quit;
GLViewer viewer;


// Sample functions
void run(vector<CameraData> &zeds, ArucoData &acuroData);
void tryReloc(CameraData &it, ArucoData &acuroData);
void displayMarker(CameraData &it, ArucoData &acuroData);
void propagateTransforms(vector<CameraData> &zeds);
void close();

int main(int argc, char **argv) {
    // Set configuration parameters
    InitParameters init_params;
    init_params.coordinate_units = UNIT::METER;
    //init_params.depth_mode = DEPTH_MODE::NEURAL;
    init_params.camera_fps = 30;

    // detect number of connected ZED camera.
    auto zed_infos = Camera::getDeviceList();
    const int nb_zeds = zed_infos.size();
    vector<CameraData> zeds(nb_zeds);

    // specify size for the point cloud
    Resolution res(512, 288);

    // define Positional Tracking parameters
    PositionalTrackingParameters tracking_params;
    tracking_params.enable_area_memory = false;
    tracking_params.set_as_static = true;

    ArucoData acuroData;
    acuroData.dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_100);
    acuroData.marker_length = 0.13f; // Size real size of the maker in meter

    cout << "Make sure the ArUco marker is a 6x6 (100), measuring " << acuroData.marker_length * 1000 << " mm" << endl;

    int nb_zed_open = 0;
    // try to open all  the connected cameras

    for (int i = 0; i < nb_zeds; i++) {
        init_params.input.setFromCameraID(zed_infos[i].id);
        ERROR_CODE err = zeds[i].zed.open(init_params);
        if (err == ERROR_CODE::SUCCESS) {
            nb_zed_open++;
            zeds[i].id = i;
            zeds[i].point_cloud.alloc(res, MAT_TYPE::F32_C4, MEM::GPU);
            // retrieve camera calibration information for aruco detection
            auto camCalib = zeds[i].zed.getCameraInformation().camera_configuration.calibration_parameters.left_cam;
            zeds[i].camera_matrix(0, 0) = camCalib.fx;
            zeds[i].camera_matrix(1, 1) = camCalib.fy;
            zeds[i].camera_matrix(0, 2) = camCalib.cx;
            zeds[i].camera_matrix(1, 2) = camCalib.cy;

            zeds[i].im_left.alloc(camCalib.image_size, MAT_TYPE::U8_C4);
            zeds[i].im_left_ocv = cv::Mat(zeds[i].im_left.getHeight(), zeds[i].im_left.getWidth(), CV_8UC4, zeds[i].im_left.getPtr<sl::uchar1>(MEM::CPU));
            zeds[i].im_left_ocv_rgb = cv::Mat(zeds[i].im_left.getHeight(), zeds[i].im_left.getWidth(), CV_8UC3);

            // start Positional tracking (will be reset with the marker position)            
            zeds[i].zed.enablePositionalTracking(tracking_params);

            zeds[i].info = "[" + std::to_string(i) + "] " + std::string(toString(zeds[i].zed.getCameraInformation().camera_model).c_str()) + " SN" + std::to_string(zeds[i].zed.getCameraInformation().serial_number);

            cout << "Opening " << zeds[i].info << endl;
        }
    }

    if (nb_zed_open != nb_zeds) {
        cout << "Error: Number of ZED open: " << nb_zed_open << " vs " << nb_zeds << " detected.\n";
        return 1;
    }

    if (nb_zed_open == 0) {
        cout << "Error: No ZED detected\n";
        return 1;
    }

    // initialise 3D viewer
    viewer.init(res.width, res.height, nb_zeds);

    // Start the camera thread
    quit = false;
    thread zed_callback(run, ref(zeds), ref(acuroData));

    // Set the display callback
    glutCloseFunc(close);
    glutMainLoop();

    // Stop callback
    if (zed_callback.joinable())
        zed_callback.join();

    // Free buffer and close the ZEDs
    for (auto &it : zeds)
        it.point_cloud.free();

    for (auto &it : zeds)
        it.zed.close();
    return 0;
}

/**
        This function calculates the positions of all the cameras with respect to the first one
 **/
void propagateTransforms(vector<CameraData> &zeds) {
    sl::Transform ref_aruco;
    sl::Transform ref_cam_pose_gravity;
    bool first_cam = true;

    std::vector<FusionConfiguration> cam_config;
    for (auto &it : zeds) {       
        FusionConfiguration config; 
        if(first_cam) {
            ref_aruco = it.aruco_transf;
            sl::Pose cam_pose;
            it.zed.getPosition(cam_pose);
            ref_cam_pose_gravity = cam_pose.pose_data;
            first_cam = false;
        } else {
            sl::Transform this_to_ref = sl::Transform::inverse(ref_aruco) * it.aruco_transf;
            sl::Transform cam_pose_gravity = ref_cam_pose_gravity * this_to_ref;
            it.zed.resetPositionalTracking(cam_pose_gravity);
            // the IMU is direclty applied by the SDK
            config.pose = this_to_ref;
        }
        // set camera configuration
        config.communication_parameters.setForSharedMemory();
        config.serial_number = it.zed.getCameraInformation().serial_number;
        config.input_type.setFromSerialNumber(config.serial_number);
        cam_config.push_back(config);
    }

    // export camera configuration to use fusion module
    sl::writeConfigurationFile("MultiCamConfig.json", cam_config, COORDINATE_SYSTEM::IMAGE, UNIT::METER);
}

/**
        This function loops to get image and motion data from the ZED. It is similar to a callback.
        Add your own code here.
 **/
void run(vector<CameraData> &zeds, ArucoData &acuroData) {
    RuntimeParameters rt_p;
    // ask for point cloud in World Frame, then they will be displayed with the same reference
    rt_p.measure3D_reference_frame = REFERENCE_FRAME::WORLD;
    rt_p.confidence_threshold = 10;

    bool not_aligned = true;

    while (!quit) {
        bool all_cam_located = true;
        for (auto &it : zeds) { // for all camera
            if (it.zed.grab(rt_p) == ERROR_CODE::SUCCESS) { // grab new image	
                it.zed.retrieveImage(it.im_left);
                displayMarker(it, acuroData);
                if (!it.is_reloc)  // if still ne relocated, grab left image and look for ArUco pattern
                    tryReloc(it, acuroData);                
                all_cam_located = all_cam_located & it.is_reloc;    
            }
        }

        if(all_cam_located){
            if(not_aligned) {
                propagateTransforms(zeds);
                not_aligned = false;
            }

            for (auto &it : zeds) { // for all camera
                it.zed.retrieveMeasure(it.point_cloud, MEASURE::XYZRGBA, MEM::GPU, it.point_cloud.getResolution());
                viewer.updatePointCloud(it.point_cloud, it.id);
            }
        }


        if (viewer.askReset()) { // you can force a new reloc by pressing 'space' from the 3D view;
            cout << "Reset asked\n";
            all_cam_located = false;
            not_aligned = true;
            for (auto &it : zeds){
                it.is_reloc = false;
                sl::Transform path_identity;
                it.zed.resetPositionalTracking(path_identity);
            }
        }
        sleep_ms(1);
    }
}

void tryReloc(CameraData &it, ArucoData &acuroData) {
    vector<int> ids;
    vector<vector<cv::Point2f> > corners;

    cv::cvtColor(it.im_left_ocv, it.im_left_ocv_rgb, cv::COLOR_RGBA2RGB);
    aruco::detectMarkers(it.im_left_ocv_rgb, acuroData.dictionary, corners, ids);

    if (ids.size()) { // an ArUco marker has been detected
        vector<cv::Vec3d> rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(corners, acuroData.marker_length, it.camera_matrix, it.dist_coeffs, rvecs, tvecs);

        //compute a Transform based on the first marker data
        it.aruco_transf.setTranslation(sl::float3(tvecs[0](0), tvecs[0](1), tvecs[0](2)));
        it.aruco_transf.setRotationVector(sl::float3(rvecs[0](0), rvecs[0](1), rvecs[0](2)));        
        it.aruco_transf.inverse();
        it.is_reloc = true; 
    }
}

void displayMarker(CameraData &it, ArucoData &acuroData) {
    vector<int> ids;
    vector<vector<cv::Point2f> > corners;

    cv::cvtColor(it.im_left_ocv, it.im_left_ocv_rgb, cv::COLOR_RGBA2RGB);
    aruco::detectMarkers(it.im_left_ocv_rgb, acuroData.dictionary, corners, ids);

    if (ids.size()) { // an ArUco marker has been detected
        vector<cv::Vec3d> rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(corners, acuroData.marker_length, it.camera_matrix, it.dist_coeffs, rvecs, tvecs);

        // draw the marker
        aruco::drawDetectedMarkers(it.im_left_ocv_rgb, corners, ids, cv::Scalar(236, 188, 26));
        aruco::drawAxis(it.im_left_ocv_rgb, it.camera_matrix, it.dist_coeffs, rvecs[0], tvecs[0], acuroData.marker_length * 0.5f);
    }

    cv::imshow(it.info, it.im_left_ocv_rgb);
    cv::waitKey(10);
}

/**
        This function closes the callback (thread) and the GL viewer
 **/
void close() {
    quit = true;
}
