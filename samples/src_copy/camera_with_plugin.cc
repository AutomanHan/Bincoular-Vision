#include <iomanip>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
    //cam.SetMode(Mode::MODE_CPU);

#ifdef SDK_SAMPLES_LIB_DIR
    stringstream path;
    path << SDK_SAMPLES_LIB_DIR << "/" << LIBPLUGIN;
    cam.ActivatePlugin(path.str());
#else
#ifdef _WIN32
    cam.ActivatePlugin(Camera::GetSDKRoot() + "/samples/bin/" + LIBPLUGIN);
#else
    cam.ActivatePlugin(Camera::GetSDKRoot() + "/samples/lib/" + LIBPLUGIN);
#endif
#endif
    cam.ActivateDepthMapFeature();

    InitParameters params(name);
    cam.Open(params);

    if (!cam.IsOpened()) {
        std::cerr << "Error: Open camera failed" << std::endl;
        return 1;
    }
    std::cout << "\033[1;32mPress ESC/Q on Windows to terminate\033[0m\n";

    double t, fps = 0;
    ErrorCode code;
    cv::Mat img_left, img_right;
    cv::Mat depthmap;
    cv::Mat depthmap_color;
    for (;;) {
        t = (double)cv::getTickCount();

        code = cam.Grab();

        if (code != ErrorCode::SUCCESS) continue;

        if (cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED) == ErrorCode::SUCCESS &&
            cam.RetrieveImage(img_right, View::VIEW_RIGHT_UNRECTIFIED) == ErrorCode::SUCCESS) {

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

            cv::imshow("left", img_left);
            cv::imshow("right", img_right);
        }

        code = cam.RetrieveImage(depthmap, View::VIEW_DEPTH_MAP);
        if (code == ErrorCode::SUCCESS) {
#ifdef USE_OPENCV2
            depthmap_color = depthmap;  // do nothing
#else
            cv::applyColorMap(depthmap, depthmap_color, cv::COLORMAP_JET);
#endif
            cv::imshow("depthmap", depthmap_color);
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
