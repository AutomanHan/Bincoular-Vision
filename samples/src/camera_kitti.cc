#include <iostream>
#include <chrono>
#include <thread>

#include "camera.h"
#include "compat/compat.h"
#include "dataset/dataset.h"
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

    const char *outdir;
    if (argc >= 3) {
        outdir = argv[2];
    } else {
        outdir = "./dataset";
    }

    Camera cam;
    InitParameters params(name);
    cam.Open(params);

    if (cam.IsOpened()) {
        cout << "Open Camera: " << name << endl
            << "\033[1;32mPress any key to stop capturing.\033[0m\n";
    } else {
        cerr << "Error: Open camera failed" << endl;
        return 1;
    }

    cam.ActivateAsyncGrabFeature();

    Dataset *dataset = new KittiDataset(outdir);
    cout << "Capture dataset to directory:\n  " << outdir << endl;

    using clock = chrono::steady_clock;
    clock::time_point time_start = clock::now();

    ErrorCode code;
    cv::Mat img_left, img_right;
    for (;;) {
        code = cam.Grab();

        if (code != ErrorCode::SUCCESS) continue;

        if (cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED) == ErrorCode::SUCCESS &&
            cam.RetrieveImage(img_right, View::VIEW_RIGHT_UNRECTIFIED) == ErrorCode::SUCCESS) {

            dataset->Save(img_left, img_right);
        }

        // Sleep to recude the fps from 60 to 20
        //   time ≈ 1000/20 - 1000/60 = 50 - 16.67 = 33.33 ms
        this_thread::sleep_for(chrono::milliseconds(33));

        if (_kbhit()) break;
    }

    cam.Close();

    double seconds = chrono::duration_cast<chrono::duration<double>>(
        clock::now() - time_start).count();
    cout << "\n  Cost " << seconds << " seconds, average "
        << dataset->GetCount() / seconds << " fps" << endl;

    cout << "\033[1;32mKITTI dataset, captured "
        << dataset->GetCount() << " frames in total.\033[0m" << endl;

    delete dataset;
    return 0;
}
