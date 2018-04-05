#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

#define LEN 128

int main()
{
	VideoCapture cap(1);
	if(!cap.isOpened())
	{
		return -1;
	}
	
	bool stop = false;
	Mat frame;
	int i = 1;
	char SaveName[LEN];
	while(!stop)
	{
		
		cap >> frame;
		imshow("当前图像", frame);
		if(waitKey(1) == 48)
		{
			sprintf(SaveName, "../picturepath_cam2/%5d.jpg", i++);
			imwrite(SaveName, frame);

		}
		if(waitKey(1) == 27)
		{
			stop = true;
		} 
	}

	return 0;
}