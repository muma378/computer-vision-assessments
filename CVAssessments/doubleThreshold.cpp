//
//  main.cpp
//  CVAssessments
//
//  Created by Xiao Yang on 15/3/30.
//  Copyright (c) 2015年 Young. All rights reserved.
//

#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/video/video.hpp>

using namespace std;
using namespace cv;
int max_val = 255;
int min_val = 0;
Mat edge;

void doubleThreshold(void);


int main(int argc, const char * argv[]) {
    VideoCapture cap("/Users/imac/Desktop/Computer Vision/videos/ball.mp4");
//    VideoCapture cap("/Users/imac/Download/ball.avi");
//    cap.set(CAP_PROP_POS_AVI_RATIO, 0.5);
    if(!cap.isOpened())
        return -1;
    
    namedWindow("Edges");
    while (1) {
        Mat frame;
        frame = imread("/Users/imac/Desktop/Computer Vision/SWAN.jpg");
//        if (!cap.read(frame)) {
//            std::cout << "Unable to retrieve frame from video stream." << std::endl;
//            continue;
//        }
//        cout << cap.get(CAP_PROP_FRAME_HEIGHT) << "*" << cap.get(CAP_PROP_FRAME_WIDTH) << endl;
//        cout << frame.rows << "*" << frame.cols << endl;
        cvtColor(frame, edge, COLOR_RGB2GRAY);
        GaussianBlur(edge, edge, Size(7, 7), 1.5, 1.5);
        
        inRange(edge, 160, 230, edge);
//        doubleThreshold();
//        Canny(edge, edge, 0, 30, 3);
//        threshold(edge, edge, 80, 255, THRESH_BINARY);
//        if (frame.empty()) break;
        imshow("edges", edge);
        if (waitKey(30)>=0) break;
    }
    return 0;
    
}

void bandFilter(int, void*){
    inRange(edge, Scalar(min_val), Scalar(max_val), edge);
    imshow("Edges", edge);

}


void doubleThreshold( void )
{
    
    createTrackbar("MinVal", "Edges", &min_val, 255, bandFilter);
    createTrackbar("MaxVal", "Edges", &max_val, 255, bandFilter);
    
    bandFilter(0, 0);
}

