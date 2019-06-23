//
// Created by hanc on 19-4-25.
//

#include <iostream>
//#include <opencv2/highgui.hpp>
//#include "opencv2/core.hpp"
//#include "opencv2/imgproc.hpp"
//#include "mynteye/api/api.h"
#include "bi_cam.h"
int main(){
    bi_cam cam_bino;
    std::string imglist="../imglist/stereo_calib.xml";
    cam_bino.calib_bino(7,5,33,imglist);
    return 0;
}
