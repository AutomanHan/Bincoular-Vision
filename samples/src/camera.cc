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
        std::cerr << "Error: Open camera failed" << std::endl;
        return 1;
    }
    std::cout << "\033[1;32mPress ESC/Q on Windows to terminate\033[0m\n";

    double t, fps = 0;
    ErrorCode code;
    cv::Mat img_left, img_right;
    int count_num = 0;
    char left_name[100], right_name[100];
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

        char key = (char) cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            break;
        }
	if (key =='s' || key == 'S'){
	    count_num ++;
	    sprintf(left_name,"../imglist/left%02d.jpg", count_num);
	    sprintf(right_name, "../imglist/right%02d.jpg", count_num);
	    cv::imwrite(left_name, img_left);
	    cv::imwrite(right_name, img_right);
	    
	}

        t = (double)cv::getTickCount() - t;
        fps = cv::getTickFrequency() / t;
    }

    cam.Close();
    cv::destroyAllWindows();
    return 0;
}
