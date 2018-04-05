#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	VideoCapture inputVideo(1);
	if(!inputVideo.isOpened())
	{
		cout<<"Could not open the input video:"<<endl;
		return -1;
	}
	Mat frame;
	Mat frameCalibration;

	inputVideo>>frame;
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<double>(0, 0) = 843.9546;
	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(0, 2) = 288.5969;
	cameraMatrix.at<double>(1, 1) = 844.0386;
	cameraMatrix.at<double>(1, 2) = 221.0515;

	Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
	distCoeffs.at<double>(0, 0) = -0.4239;
	distCoeffs.at<double>(1, 0) = 0.2188;
	distCoeffs.at<double>(2, 0) = 0;
	distCoeffs.at<double>(3, 0) = 0;
	distCoeffs.at<double>(4, 0) = 0;


	Mat view, rview, map1, map2;
	Size imageSize;
	imageSize = frame.size();
	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
		getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
		imageSize, CV_16SC2, map1, map2);
	cout<<frame.size()<<endl;
	cout<<map1.size()<<endl;


	while(1)  //show the image captured in the window and repeat
	{
		inputVideo >> frame;
		if(frame.empty()) break;
		remap(frame, frameCalibration, map1, map2, INTER_LINEAR);
		imshow("原始图像", frame);
		imshow("校正后图像", frameCalibration);
		char key = waitKey(1);
		if (key == 27 || key == 'q' || key == 'Q') break;
	}
	return 0;
}
