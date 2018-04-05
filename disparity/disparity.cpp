#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>



#include <iostream>

using namespace cv;
using namespace std;

int main()
{
// 	VideoCapture Video1(0);
// 	VideoCapture Video2(1);
// 	if(!Video1.isOpened())
// 	{
// 		cout<<"Could not open the input video1."<<endl;
// 		return 0;
// 	}
// 	if (!Video2.isOpened())
// 	{
// 		cout<<"Could not open the input video2."<<endl;
// 		return 0;
// 	}
	
// 	double minVal; double maxVal;
// 	int ndisparity = 16*1;
// 	int SADWindowSize = 7;

// 	Ptr<StereoBM> sbm = StereoBM::create(ndisparity, SADWindowSize);
// 	sbm->setPreFilterCap(CV_STEREO_BM_NORMALIZED_RESPONSE);
// 	sbm->setPreFilterSize(21);
// 	sbm->setPreFilterCap(31);
// 	sbm->setBlockSize(21);
// 	sbm->setMinDisparity(-16);
// 	sbm->setNumDisparities(48);
// 	sbm->setTextureThreshold(10);
// 	sbm->setUniquenessRatio(8);
// 	sbm->setSpeckleWindowSize(10);
// 	sbm->setSpeckleRange(32);

// 	Mat frame1, frame2;
// 	Video1 >> frame1;
// 	Video2 >> frame2;
// 	Mat imgDisparity16S = Mat(frame1.rows, frame1.cols, CV_16S);
// 	Mat imgDisparity8U = Mat(frame2.rows, frame2.cols, CV_8UC1);
// 	//矫正
// 	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
// 	cameraMatrix.at<double>(0, 0) = 843.9546;
// 	cameraMatrix.at<double>(0, 1) = 0;
// 	cameraMatrix.at<double>(0, 2) = 288.5969;
// 	cameraMatrix.at<double>(1, 1) = 844.0386;
// 	cameraMatrix.at<double>(1, 2) = 221.0515;

// 	Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
// 	distCoeffs.at<double>(0, 0) = -0.4239;
// 	distCoeffs.at<double>(1, 0) = 0.2188;
// 	distCoeffs.at<double>(2, 0) = 0;
// 	distCoeffs.at<double>(3, 0) = 0;
// 	distCoeffs.at<double>(4, 0) = 0;


// 	Mat view, rview, map1_1, map1_2, map2_1, map2_2;
// 	Size imageSize;
// 	imageSize = frame1.size();
// 	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
// 		getOptimalNewCameraMatri(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
// 		imageSize, CV_16SC2, map1_1, map1_2);
// 	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
// 		getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
// 		imageSize, CV_16SC2, map2_1, map2_2);

// ///////////////////////////////////////////////////////////////////////////////	


// 	while(1)
// 	{
// 		Video1 >> frame1;
		
// 		Video2 >> frame2;
		
// 		remap(frame1, frame1, map1_1, map1_2, INTER_LINEAR);
// 		remap(frame2, frame2, map2_1, map2_2, INTER_LINEAR);

// 		cvtColor(frame1, frame1, COLOR_BGR2GRAY); 
// 		cvtColor(frame2, frame2, COLOR_BGR2GRAY); 
// 		imshow("Video1", frame1);
// 		imshow("Video2", frame2);
// 		sbm->compute(frame1, frame2, imgDisparity16S);
		
// 		minMaxLoc(imgDisparity16S, &minVal, &maxVal);
// 		imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255/(maxVal-minVal));
// 		imshow("disparity", imgDisparity8U);
// 		char key = waitKey(1);
// 		if(key == 'q' || key == 'Q') break;

// 	}

	Mat imgLeft;
	imgLeft = imread("../imL.png",IMREAD_GRAYSCALE);
	Mat imgRight; 
	imgRight = imread("../imR.png", IMREAD_GRAYSCALE);

	//CvSize size = cvGetSize(image_L);

	//cout<<imgLeft.size().width<<endl;
	//cout<<image_R.shape(1)<<endl;
	Mat disparity_left(imgLeft.size(), CV_16S);
	Mat disparity_right(imgRight.size(), CV_16S);
	Mat imgDisparity16S = Mat(imgLeft.rows, imgRight.cols, CV_16S);
	Mat imgDisparity8U = Mat(imgRight.rows, imgRight.cols, CV_8UC1);

	//2.Call the constructor for StereoBM
	int ndisparity = 16*5;
	int SADWindowSize = 7;

	Ptr<StereoBM> sbm = StereoBM::create(ndisparity, SADWindowSize);

	Rect roi_left, roi_right;

	//3.Calculate teh disparity image
	sbm->setPreFilterCap(CV_STEREO_BM_NORMALIZED_RESPONSE);
	sbm->setPreFilterSize(5);
	sbm->setPreFilterCap(31);
	sbm->setBlockSize(21);
	sbm->setMinDisparity(-16);
	sbm->setNumDisparities(80);
	sbm->setTextureThreshold(10);
	sbm->setUniquenessRatio(8);
	sbm->setSpeckleWindowSize(10);
	sbm->setSpeckleRange(32);
	// Ptr<StereoSGBM> sbm = StereoSGBM::create();
	sbm->compute(imgLeft, imgRight, imgDisparity16S);

	double minVal; double maxVal;
	minMaxLoc(imgDisparity16S, &minVal, &maxVal);

	cout<<"min disp: %f"<< minVal<<"  "<<"max disp: %f" <<maxVal<<endl;

	imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255/(maxVal-minVal));
	imshow("disparity", imgDisparity8U);
	//namedWindow(windowDisparity, WINDOW_NORMAL);
	//imshow(windowDisparity, imgDisparity8U);

	imshow("left", imgLeft);
	imshow("right", imgRight);
	while(1)
	{
		char key = waitKey(1);
		if(key == 'q' || key == 'Q')
			break;
	}

	// int ndisparity = 16*5;
 // 	int SADWindowSize = 7;
	// Ptr<StereoSGBM> sgbm = StereoSGBM::create(ndisparity, SADWindowSize);
	// StereoSGBM sgbm;
 //    int SADWindowSize = 9;
 //    sgbm.preFilterCap = 63;
 //    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
 //    int cn = img1->nChannels;
 //    int numberOfDisparities=64;
 //    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
 //    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
 //    sgbm.minDisparity = 0;
 //    sgbm.numberOfDisparities = numberOfDisparities;
 //    sgbm.uniquenessRatio = 10;
 //    sgbm.speckleWindowSize = 100;
 //    sgbm.speckleRange = 32;
 //    sgbm.disp12MaxDiff = 1;
 //    Mat disp, disp8;
 //    int64 t = getTickCount();
 //    sgbm(imL, imR, disp);
 //    t = getTickCount() - t;
 //    cout<<"Time elapsed:"<<t*1000/getTickFrequency()<<endl;
 //    disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));

 //   imshow("dispartiy", disp8);

	return 0;

}
