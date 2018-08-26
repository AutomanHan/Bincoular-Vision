#include <iomanip>
#include <iostream>
#include <mutex>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/calib3d.hpp"

#include "camera.h"
#include "utility.h"
using namespace cv;

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

}  // namespace

using namespace std;
using namespace mynteye;

Mat histgram(Mat src_img)
{
    int row_num = src_img.rows;
    int col_num = src_img.cols;
    Mat dst_img(row_num, col_num, CV_8UC1, Scalar(0,0,0));
    
    double hist[256] = {0.00};
    double dhist[256] = {0.00};
    double Dhist[256] = {0.00};

    for (int i= 0; i< row_num;i++)
    {
	uchar *data = src_img.ptr<uchar>(i);
	for(int j = 0; j<col_num; j++)
	{
	    int temp = data[j];
	    hist[temp] += 1;	
	}
    }

    for(int i = 0;i<256; i++)
    {
        dhist[i] = hist[i]/(row_num*col_num);
	if(0==i)
	{
	    Dhist[i] = dhist[i];	
	}
	else
	{
	   Dhist[i] = Dhist[i-1] + dhist[i];
	}
	
    }
   // cout<<"Dhist[255]:"<<Dhist[255]<<endl;

    for(int i=0; i<row_num; i++)
    {
	uchar* data1 = dst_img.ptr<uchar>(i);
	uchar* data2 = src_img.ptr<uchar>(i);
	for(int j = 0; j< col_num; j++)
	{
	    int temp1 = data2[j];
	    int temp2 = (int)(Dhist[temp1]*255)	;
	    data1[j] = temp2;
	}
    }
    return dst_img;
}



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
    //cam.ActivatePointCloudFeature();

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

    
    cv::Mat depthmap, pointcloud;
    cv::Mat depthmap_color;

    vector<IMUData> imudatas;
    std::uint32_t timestamp;

    std::uint64_t count = 0;
    std::uint64_t depthmap_count = 0;
	
    FileStorage fs_ex, fs_in;
    Mat M1,M2,D1,D2;
    Mat P1,P2,R1,R2;
    Mat mapl1, mapl2;
    Mat mapr1, mapr2;
    fs_ex.open("/home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/calib/extrinsics.yml",FileStorage::READ);
    fs_in.open("/home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/calib/intrinsics.yml",FileStorage::READ);
    fs_in["M1"]>>M1;fs_in["M2"]>>M2;fs_in["D1"]>>D1;fs_in["D2"]>>D2;
    fs_ex["R1"]>>R1;fs_ex["R2"]>>R2;fs_ex["P1"]>>P1;fs_ex["P2"]>>P2;
    cout<<"M1:"<<M1<<endl;
    
    code = cam.Grab();
    char name_window[20]="disparity";
    namedWindow(name_window, WINDOW_AUTOSIZE);
    cv::Mat img_left, img_right;
    //createTrackbar("ndisparity:", name_window,&nd, 10, onTrackbarSlide);
    //createTrackbar("SADWindowSize", name_window, &sadwindowsize, 50, onTrackbarSlide);
    //createTrackbar("mindisparity:", name_window, &mindis, 50, onTrackbarSlide);

    while(1)
    {
	if (cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED) == ErrorCode::SUCCESS &&
            cam.RetrieveImage(img_right, View::VIEW_RIGHT_UNRECTIFIED) == ErrorCode::SUCCESS)    
        {
	    break;
	}
    }    
	int nd=5;
int sadwindowsize=15;
int mindis=0;
int mindisparity =mindis;
int ndisprities = 16*nd;
int SADWindowSize = sadwindowsize;

Size img_size = img_left.size();
int numberOfDisparities = ((img_size.width/8) + 15) & -16;
Ptr<StereoSGBM> sgbm = StereoSGBM::create(mindisparity, ndisprities, SADWindowSize);

    mindisparity =0;
    ndisprities = 16*nd;
    SADWindowSize = sadwindowsize;


    //Mat disp;
    
    //SGBM
   
    int P1_sgbm = 8*img_left.channels()*SADWindowSize*SADWindowSize;
    int P2_sgbm = 32*img_left.channels()*SADWindowSize*SADWindowSize;
    
    sgbm->setP1(P1_sgbm);
    sgbm->setP2(P2_sgbm);

    sgbm->setPreFilterCap(31);  
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(25);  
    sgbm->setSpeckleRange(32);  
    sgbm->setSpeckleWindowSize(100);  
    sgbm->setDisp12MaxDiff(1);      
    sgbm->setMode(StereoSGBM::MODE_SGBM);

	//int numberOfDisparities=5;
    Mat disparity_left(img_left.size(), CV_16S);
    Mat disparity_right(img_right.size(), CV_16S);
    Mat imgDisparity16S = Mat(img_left.rows, img_right.cols, CV_16S);
    Mat imgDisparity8U = Mat(img_right.rows, img_right.cols, CV_8UC1);
    /*
//2.Call the constructor for StereoBM
	int ndisparity = 16;
	int SADWindowSize = 7;
	Ptr<StereoBM> sbm = StereoBM::create(ndisparity, SADWindowSize);
    //3.Calculate teh disparity image
	sbm->setPreFilterCap(CV_STEREO_BM_NORMALIZED_RESPONSE);
	sbm->setPreFilterSize(5);
	sbm->setPreFilterCap(31);
	sbm->setBlockSize(21);
	sbm->setMinDisparity(-16);
	sbm->setNumDisparities(48);
	sbm->setTextureThreshold(10);
	sbm->setUniquenessRatio(15);
	sbm->setSpeckleWindowSize(10);
	sbm->setSpeckleRange(32);

//Ptr<StereoBM> sbm = StereoBM::create(ndisparity, SADWindowSize);
    //initUndistortRectifyMap(M1, D1, Mat(),P1,img_left.size(), CV_16SC2, mapl1, mapl2);
    //initUndistortRectifyMap(M2, D2, R2,P2,img_right.size(), CV_16SC2, mapr1, mapr2);	
    //initUndistortRectifyMap(M1, D1, Mat(),P1,img_left.size(), CV_16SC2, mapl1, mapl2);
    //initUndistortRectifyMap(M2, D2, Mat(),P2,img_right.size(), CV_16SC2, mapr1, mapr2);
*/
    


    

cv::fisheye::initUndistortRectifyMap(M1, D1, R1, P1, img_right.size(), CV_16SC2, mapl1, mapl2);
    cv::fisheye::initUndistortRectifyMap(M2, D2, R2, P2, img_right.size(), CV_16SC2, mapr1, mapr2);
    
    Mat img_l_c,img_r_c;
	Mat roi_l, roi_r;
	
	Rect rect_roi(30,30, img_size.width-30, img_size.height-100);
	//cout << "width:" <<rect_roi.width<<endl;
	//cout << "height:"<<rect_roi.height<<endl;
	Mat hist_img_l,hist_img_r;
	Mat canvas(img_size.height+20, img_size.width*2+20,  CV_8UC3);
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
	    remap(img_left, img_l_c, mapl1,mapl2, INTER_LINEAR);
	    remap(img_right, img_r_c, mapr1, mapr2, INTER_LINEAR);
	    Mat img_l_3, img_r_3;
	    hist_img_l = histgram(img_l_c);
	    hist_img_r = histgram(img_r_c);
	    cvtColor(hist_img_l, img_l_3, COLOR_GRAY2BGR);
	    cvtColor(hist_img_r, img_r_3, COLOR_GRAY2BGR);
	    cout<<canvas.cols<<"\t"<<canvas.rows<<endl;
	    Mat canvas_l = canvas(Rect(0,0,img_size.width, img_size.height));
	    cout<<canvas_l.size().width<<"\t"<<canvas_l.size().height<<endl;
	    Mat canvas_r = canvas(Rect(img_size.width,0, img_size.width, img_size.height));
	    cout<<canvas_r.size().width+50<<"\t"<<canvas_r.size().height<<endl;
	    img_l_3.copyTo(canvas_l);
	    img_r_3.copyTo(canvas_r);
	    cout<< img_l_3.cols<<"\t"<<img_l_3.rows<<endl;
	    cout<< img_r_3.cols<<"\t"<<img_r_3.rows<<endl;
	    
		line(canvas, Point(0,50), Point(img_size.width*2,50), Scalar(0,255,0), 1,8 );
	    imshow("canvas:",canvas);
	     
	    //imshow("canvas:", canvas);
	    //imshow("hist", hist_img);
	    double minVal; double maxVal;
	    //
	    //imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255/(maxVal-minVal));
		//cout<<"left size: %d, right size: %d" <<img_l_c.size()<<img_r_c.size()<<endl;
		//roi_l = hist_img_l(rect_roi);
		//roi_r = hist_img_r(rect_roi);
		sgbm->compute(hist_img_l, hist_img_r, imgDisparity16S);
		minMaxLoc(imgDisparity16S, &minVal, &maxVal);
		imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255/(maxVal-minVal));
		//sgbm->compute(roi_l, roi_r, disp);
	    imshow(name_window, imgDisparity8U);
            cv::imshow("left", img_left);
            cv::imshow("right", img_right);
	    cv::imshow("ca_left:", hist_img_l);
	    cv::imshow("cal_right:", hist_img_r);
        };
/*
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

                cv::imshow("depthmap", depthmap_color);
            }
        }
        */
/*
        code = cam.RetrieveImage(pointcloud, View::VIEW_POINT_CLOUD);
        if (code == ErrorCode::SUCCESS) {
            // TODO: Process point cloud here.
		imshow("pointcloud", pointcloud);
        }
  */      

        char key = (char) cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            break;
        }

        t = (double)cv::getTickCount() - t;
        fps = cv::getTickFrequency() / t;
    }

    cout << "Close Camera" << endl;
    cam.Close();
    cv::destroyAllWindows();
    return 0;
}
