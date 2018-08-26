#include <iomanip>
#include <iostream>
#include <mutex>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camera.h"
#include "utility.h"

namespace {

class GrabCallbacks {
public:
    GrabCallbacks()
        : tick_beg_(0),
          fps_(0),
          count_(0) {
        tick_beg_ = static_cast<double>(cv::getTickCount());
    }

    ~GrabCallbacks() = default;

    void OnPre() {}

    void OnPost(cv::Mat &left, cv::Mat &right) {
        std::lock_guard<std::mutex> lk(mtx_);
        double elapsed = (static_cast<double>(cv::getTickCount()) - tick_beg_) / cv::getTickFrequency();
        ++count_;
        fps_ = count_ / elapsed;
    }

    double GetFPS() {
        std::lock_guard<std::mutex> lk(mtx_);
        return fps_;
    }

    std::uint64_t GetCount() {
        std::lock_guard<std::mutex> lk(mtx_);
        return count_;
    }

private:
    double tick_beg_;
    double fps_;
    std::uint64_t count_;
    std::mutex mtx_;
};

class DepthMapCallbacks {
public:
    DepthMapCallbacks()
        : tick_beg_(0),
          tick_end_(0),
          time_cost_(0),
          time_total_(0),
          count_(0) {
    }

    ~DepthMapCallbacks() = default;

    void OnPre(cv::Mat &left, cv::Mat &right) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            tick_beg_ = static_cast<double>(cv::getTickCount());
        }
    }

    void OnPost(cv::Mat &depthmap) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            tick_end_ = static_cast<double>(cv::getTickCount());
            time_cost_ = (tick_end_ - tick_beg_) / cv::getTickFrequency();

            time_total_ += time_cost_;
            ++count_;
        }
    }

    double GetTimeCost() {
        std::lock_guard<std::mutex> lk(mtx_);
        return time_cost_;
    }

    double GetTimeAverage() {
        std::lock_guard<std::mutex> lk(mtx_);
        return time_total_ / count_;
    }

    std::uint64_t GetCount() {
        std::lock_guard<std::mutex> lk(mtx_);
        return count_;
    }

private:
    double tick_beg_;
    double tick_end_;
    double time_cost_;   // in seconds
    double time_total_;  // in seconds

    std::uint64_t count_;

    std::mutex mtx_;
};

std::unique_ptr<std::ios> GetStreamFormat(int wide, int prec, char fillch = ' ') {
    auto fmt = std::unique_ptr<std::ios>{new std::ios{nullptr}};
    fmt->setf(std::ios::fixed);
    fmt->width(std::move(wide));
    fmt->precision(std::move(prec));
    fmt->fill(std::move(fillch));
    return fmt;
}

template<typename T>
std::stringstream &Append(std::stringstream &ss, T text,
        std::ios *fmt = nullptr, bool reset = false) {
    if (reset) {
        ss.str("");
        ss.clear();
    }
    if (fmt) {
        ss.copyfmt(*fmt);
    }
    ss << std::move(text);
    return ss;
}

template<typename T>
std::stringstream &Append(std::stringstream &ss, T text,
        const std::unique_ptr<std::ios> &fmt, bool reset = false) {
    return Append(ss, std::move(text), fmt.get(), reset);
}

class DepthMapRegion {
public:
    explicit DepthMapRegion(std::uint32_t n)
        : n_(std::move(n)),
          show_(false),
          selected_(false),
          point_(0, 0) {
    }

    ~DepthMapRegion() = default;

    void OnMouse(const int &event, const int &x, const int &y, const int &flags) {
        if (event != CV_EVENT_MOUSEMOVE && event != CV_EVENT_LBUTTONDOWN) {
            return;
        }
        show_ = true;

        if (event == CV_EVENT_MOUSEMOVE) {
            if (!selected_) {
                point_.x = x;
                point_.y = y;
            }
        } else if (event == CV_EVENT_LBUTTONDOWN) {
            selected_ = true;
            point_.x = x;
            point_.y = y;
        }
    }

    template<typename T>
    void Show(const cv::Mat &image,
            std::function<std::string(const T &elem)> elem2string,
            int elem_space = 40) {
        // depthmap: CV_8UC1
        // xyz: Output 3-channel floating-point image of the same size as disparity
        if (!show_) return;

        int space = std::move(elem_space);
        int n = 2 * n_ + 1;
        cv::Mat im(space*n, space*n, CV_8UC3, cv::Scalar(255,255,255));

        int x, y;
        std::string str;
        int baseline = 0;
        for (int i = -n_; i <= n; ++i) {
            x = point_.x + i;
            if (x < 0 || x >= image.cols) continue;
            for (int j = -n_; j <= n; ++j) {
                y = point_.y + j;
                if (y < 0 || y >= image.rows) continue;

                str = elem2string(image.at<T>(y, x));

                cv::Scalar color(0,0,0);
                if (i == 0 && j == 0) color = cv::Scalar(0,0,255);

                cv::Size sz = cv::getTextSize(str,
                    cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

                cv::putText(im, str,
                    cv::Point((i+n_)*space + (space-sz.width)/2,
                        (j+n_)*space + (space+sz.height)/2),
                    cv::FONT_HERSHEY_PLAIN, 1, color, 1);
            }
        }

        cv::imshow("region", im);
    }

    void DrawPoint(const cv::Mat &im) {
        if (!show_) return;
        std::uint32_t n = (n_ > 1) ? n_ : 1;
#ifdef USE_OPENCV2
        cv::rectangle(const_cast<cv::Mat&>(im),
#else
        cv::rectangle(im,
#endif
            cv::Point(point_.x-n, point_.y-n),
            cv::Point(point_.x+n, point_.y+n),
            cv::Scalar(0,0,255), 1);
    }

private:
    std::uint32_t n_;
    bool show_;
    bool selected_;
    cv::Point point_;
};

void OnDepthMapMouseCallback(int event, int x, int y, int flags, void *userdata) {
    DepthMapRegion *region = reinterpret_cast<DepthMapRegion*>(userdata);
    region->OnMouse(event, x, y, flags);
}

}  // namespace

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

    CalibrationParameters *calib_params = nullptr;
    if (argc >= 3) {
        stringstream ss;
        ss << argv[2];
        calib_params = new CalibrationParameters;
        calib_params->Load(ss.str());
    }
    InitParameters init_params(name, calib_params);

    Camera cam;
    //cam.SetMode(Mode::MODE_CPU);
    // Could test plugin here.
    cam.Open(init_params);

    if (calib_params)
        delete calib_params;

    if (!cam.IsOpened()) {
        std::cerr << "Error: Open camera failed" << std::endl;
        return 1;
    }
    cout << "\033[1;32mPress ESC/Q on Windows to terminate\033[0m\n";

    cam.ActivateAsyncGrabFeature(true);
    cam.ActivateDepthMapFeature();
    cam.ActivatePointCloudFeature();

    using namespace std::placeholders;

    GrabCallbacks grab_callbacks;
    cam.SetGrabProcessCallbacks(nullptr,
        std::bind(&GrabCallbacks::OnPost, &grab_callbacks, _1, _2));

    DepthMapCallbacks depthmap_callbacks;
    cam.SetDepthMapProcessCallbacks(
        std::bind(&DepthMapCallbacks::OnPre, &depthmap_callbacks, _1, _2),
        std::bind(&DepthMapCallbacks::OnPost, &depthmap_callbacks, _1));

    // Scale the grabbed images.
    // cam.SetScale(0.5);
    // Or, scale after process grab
    // cam.SetGrabProcessCallbacks(nullptr, [](cv::Mat &left, cv::Mat &right) {
    //     cv::resize(left, left, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    //     cv::resize(right, right, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    // });
    // Or, scale after process rectification
    // cam.SetRectifyProcessCallbacks(nullptr, [](cv::Mat &left, cv::Mat &right) {
    //     cv::resize(left, left, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    //     cv::resize(right, right, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    // });

    auto fmt_fps = GetStreamFormat(6, 2);
    auto fmt_imu = GetStreamFormat(8, 4);
    auto fmt_time = GetStreamFormat(7, 2);
    std::stringstream ss;

    double t, fps = 0;
    ErrorCode code;

    cv::Mat img_left, img_right;
    cv::Mat depthmap, pointcloud;
    cv::Mat depthmap_color;

    DepthMapRegion depthmap_region(2);

    vector<IMUData> imudatas;
    std::uint32_t timestamp;

    std::uint64_t count = 0;
    std::uint64_t depthmap_count = 0;

    cv::namedWindow("left");
    cv::namedWindow("right");
    cv::namedWindow("depthmap");
    //cv::namedWindow("region");

    for (;;) {
        t = (double)cv::getTickCount();

        code = cam.Grab();

        if (code != ErrorCode::SUCCESS) continue;

        if (cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED) == ErrorCode::SUCCESS &&
            cam.RetrieveImage(img_right, View::VIEW_RIGHT_UNRECTIFIED) == ErrorCode::SUCCESS) {
            ++count;

            cam.RetrieveIMUData(imudatas, timestamp);

            // top left: width x height, count
            Append(ss, img_left.cols, nullptr, true)
                << "x" << img_left.rows << ", " << count;
            DrawInfo(img_left, ss.str(), Gravity::TOP_LEFT);
            DrawInfo(img_right, ss.str(), Gravity::TOP_LEFT);
            // top right: fps
            Append(ss, "fps:", nullptr, true);
            Append(ss, fps, fmt_fps);
            cv::Rect rect = DrawInfo(img_left, ss.str(), Gravity::TOP_RIGHT);
            DrawInfo(img_right, ss.str(), Gravity::TOP_RIGHT);
            // grab_fps
            double grab_fps = grab_callbacks.GetFPS();
            Append(ss, "grab fps:", nullptr, true);
            Append(ss, grab_fps, fmt_fps);
            DrawInfo(img_left, ss.str(), Gravity::TOP_RIGHT, 5, 0, 5 + rect.height);
            DrawInfo(img_right, ss.str(), Gravity::TOP_RIGHT, 5, 0, 5 + rect.height);

            if (!imudatas.empty()) {
                size_t size = imudatas.size();
                IMUData &imudata = imudatas[size-1];

                // bottom left: imudata
                Append(ss, "accel(x,y,z): ", nullptr, true);
                Append(ss, imudata.accel_x, fmt_imu) << ",";
                Append(ss, imudata.accel_y, fmt_imu) << ",";
                Append(ss, imudata.accel_z, fmt_imu);
                DrawInfo(img_left, ss.str(), Gravity::BOTTOM_LEFT);
                Append(ss, "gyro(x,y,z): ", nullptr, true);
                Append(ss, imudata.gyro_x, fmt_imu) << ",";
                Append(ss, imudata.gyro_y, fmt_imu) << ",";
                Append(ss, imudata.gyro_z, fmt_imu);
                DrawInfo(img_right, ss.str(), Gravity::BOTTOM_LEFT);

                /*
                cout << "IMU count: " << size << endl;
                for (size_t i = 0; i < size; ++i) {
                    auto &imudata = imudatas[i];
                    cout << "  IMU[" << i << "] time: " << (imudata.time / 10) << " ms"
                        << ", accel(" << imudata.accel_x << "," << imudata.accel_y << "," << imudata.accel_z << ")"
                        << ", gyro(" << imudata.gyro_x << "," << imudata.gyro_y << "," << imudata.gyro_z << ")"
                        << endl;
                }
                */
            }

            cv::imshow("left", img_left);
            cv::imshow("right", img_right);
        };

        auto depthmap_count_ = depthmap_callbacks.GetCount();
        if (depthmap_count != depthmap_count_) {
            depthmap_count = depthmap_count_;
            //cout << "depthmap_count: " << depthmap_count << endl;
            // only retrieve when depthmap changed
            code = cam.RetrieveImage(depthmap, View::VIEW_DEPTH_MAP);
            if (code == ErrorCode::SUCCESS) {
#ifdef USE_OPENCV2
                // `applyColorMap` provided by contrib libs in opencv 2.x
                depthmap_color = depthmap;  // do nothing
#else
                // ColormapTypes
                //   http://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html#ga9a805d8262bcbe273f16be9ea2055a65
                cv::applyColorMap(depthmap, depthmap_color, cv::COLORMAP_JET);
#endif

                // top left: count
                Append(ss, img_left.cols, nullptr, true)
                    << "x" << img_left.rows << ", " << depthmap_count;
                DrawInfo(depthmap_color, ss.str(), Gravity::TOP_LEFT);
                // top right: cost, avg
                Append(ss, "cost:", nullptr, true);
                Append(ss, depthmap_callbacks.GetTimeCost() * 1000, fmt_time) << "ms";
                cv::Rect rect = DrawInfo(depthmap_color, ss.str(), Gravity::TOP_RIGHT);
                Append(ss, "average:", nullptr, true);
                Append(ss, depthmap_callbacks.GetTimeAverage() * 1000, fmt_time) << "ms";
                DrawInfo(depthmap_color, ss.str(), Gravity::TOP_RIGHT, 5, 0, 5 + rect.height);
                // bottom left: dropped
                Append(ss, "dropped: ", nullptr, true)
                    << cam.GetDroppedCount(Process::PROC_GRAB) << ","
                    << cam.GetDroppedCount(Process::PROC_RECTIFY) << ","
                    << cam.GetDroppedCount(Process::PROC_DEPTH_MAP) << ","
                    << cam.GetDroppedCount(Process::PROC_POINT_CLOUD);
                DrawInfo(depthmap_color, ss.str(), Gravity::BOTTOM_RIGHT);

                cv::setMouseCallback("depthmap", OnDepthMapMouseCallback, &depthmap_region);

                depthmap_region.DrawPoint(depthmap_color);
                cv::imshow("depthmap", depthmap_color);

                code = cam.RetrieveImage(pointcloud, View::VIEW_POINT_CLOUD);
                if (code == ErrorCode::SUCCESS) {
                    depthmap_region.Show<cv::Vec3f>(pointcloud, [](const cv::Vec3f &elem) {
                        return std::to_string(static_cast<int>(elem[2]));
                    }, 80);
                }
            }
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
