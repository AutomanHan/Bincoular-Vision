#include <iomanip>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>

#include "camera.h"
#include "utility.h"

using namespace std;
using namespace mynteye;

int main(int argc, char const *argv[]) {
    std::string name;
    if (argc >= 2) {
        name = argv[1];
    } else {
        bool found = false;
        name = FindDeviceName(&found);
    }
    cout << "Open Camera: " << name << endl;

    Camera cam;
    InitParameters params(name);
    cam.Open(params);

    if (!cam.IsOpened()) {
        cerr << "Error: Open camera failed" << endl;
        return 1;
    }
    cout << "\033[1;32mPress ESC/Q on Windows to terminate\033[0m\n";


    // Sets framerate & IMU frequency.
    cam.SetRate(Rate::RATE_50_FPS_500_HZ);
    //cam.SetRate(Rate::RATE_25_FPS_500_HZ);
    //cam.SetRate(Rate::RATE_10_FPS_200_HZ);

    // Sets auto-exposure.
    // Note: If manual-exposure is enabled, brightness, contrast and gain could be set.
    //       If auto-exposure is enabled, desired-brightness, max-gain and max-exposure-time could be set.
    cam.SetControlProperty(ControlProperty::CT_PROP_AUTO_EXPOSURE_MODE, CT_AUTO_EXPOSURE_MODE_OFF);

    auto ae_mode = cam.GetControlProperty(ControlProperty::CT_PROP_AUTO_EXPOSURE_MODE);
    cout << "Auto-Exposure: " << (ae_mode == CT_AUTO_EXPOSURE_MODE_ON ? "true" : "false") << endl;

    // Sets brightness.
    cam.SetControlProperty(ControlProperty::CT_PROP_BRIGHTNESS, 120);  // [0,240]
    cout << "Brightness: " << cam.GetControlProperty(ControlProperty::CT_PROP_BRIGHTNESS) << endl;
    // Sets contrast.
    cam.SetControlProperty(ControlProperty::CT_PROP_CONTRAST, 127);  // [0,255]
    cout << "Contrast: " << cam.GetControlProperty(ControlProperty::CT_PROP_CONTRAST) << endl;
    // Sets gain.
    cam.SetControlProperty(ControlProperty::CT_PROP_GAIN, 24);  // [0,48]
    cout << "Gain: " << cam.GetControlProperty(ControlProperty::CT_PROP_GAIN) << endl;

    // Sets desired brightnes.
    cam.SetControlProperty(ControlProperty::CT_PROP_DESIRED_BRIGHTNESS, 127);  // [0,255]
    cout << "Desired brightness: " << cam.GetControlProperty(ControlProperty::CT_PROP_DESIRED_BRIGHTNESS) << endl;
    // Sets max exposure time.
    cam.SetControlProperty(ControlProperty::CT_PROP_MAX_EXPOSURE_TIME, 120);  // [0,240]
    cout << "MaxExposureTime: " << cam.GetControlProperty(ControlProperty::CT_PROP_MAX_EXPOSURE_TIME) << endl;
    // Sets max gain.
    cam.SetControlProperty(ControlProperty::CT_PROP_MAX_GAIN, 24);  // [0,48]
    cout << "Max gain: " << cam.GetControlProperty(ControlProperty::CT_PROP_MAX_GAIN) << endl;

    // Requests zero drift calibration.
    //cam.RequestZeroDriftCalibration();


    double t, fps = 0;
    ErrorCode code;
    cv::Mat img_left, img_right;

    vector<IMUData> imudatas;
    uint64_t img_count = 0;
    uint64_t imu_count = 0;

    double tick_beg = (double)cv::getTickCount();

    for (;;) {
        t = (double)cv::getTickCount();

        code = cam.Grab();

        if (code != ErrorCode::SUCCESS) continue;

        if (cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED) == ErrorCode::SUCCESS &&
            cam.RetrieveImage(img_right, View::VIEW_RIGHT_UNRECTIFIED) == ErrorCode::SUCCESS) {
            ++img_count;

            cam.RetrieveIMUData(imudatas);
            imu_count += imudatas.size();


            double elapsed = ((double)cv::getTickCount() - tick_beg) / cv::getTickFrequency();

            // top left: width x height
            stringstream ss;
            ss << img_left.cols << "x" << img_left.rows;
            DrawInfo(img_left, ss.str(), Gravity::TOP_LEFT);
            DrawInfo(img_right, ss.str(), Gravity::TOP_LEFT);
            // top right: fps
            ss.str(""); ss.clear();
            ss << fixed << setw(7) << setprecision(2) << setfill(' ') << fps;
            DrawInfo(img_left, ss.str(), Gravity::TOP_RIGHT);
            DrawInfo(img_right, ss.str(), Gravity::TOP_RIGHT);
            // bottom left: image
            ss.str(""); ss.clear();
            ss << "IMG CNT: " << img_count;
            cv::Rect rect = DrawInfo(img_left, ss.str(), Gravity::BOTTOM_LEFT);
            DrawInfo(img_right, ss.str(), Gravity::BOTTOM_LEFT);
            ss.str(""); ss.clear();
            ss << "IMG FPS: " << setprecision(2) << (img_count / elapsed);
            DrawInfo(img_left, ss.str(), Gravity::BOTTOM_LEFT, 5, 0, -(5 + rect.height));
            DrawInfo(img_right, ss.str(), Gravity::BOTTOM_LEFT, 5, 0, -(5 + rect.height));
            // bottom left: imu
            ss.str(""); ss.clear();
            ss << "IMU CNT: " << imu_count;
            rect = DrawInfo(img_left, ss.str(), Gravity::BOTTOM_RIGHT);
            DrawInfo(img_right, ss.str(), Gravity::BOTTOM_RIGHT);
            ss.str(""); ss.clear();
            ss << "IMU HZ: " << setprecision(2) << (imu_count / elapsed);
            DrawInfo(img_left, ss.str(), Gravity::BOTTOM_RIGHT, 5, 0, -(5 + rect.height));
            DrawInfo(img_right, ss.str(), Gravity::BOTTOM_RIGHT, 5, 0, -(5 + rect.height));

            cv::imshow("left", img_left);
            cv::imshow("right", img_right);
        }

        char key = (char) cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            break;
        }

        t = (double)cv::getTickCount() - t;
        fps = cv::getTickFrequency() / t;
    }

    cam.Close();
    cv::destroyAllWindows();
    return 0;
}
