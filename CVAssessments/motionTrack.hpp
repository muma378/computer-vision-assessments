//
//  motionTrack.h
//  CVAssessments
//
//  Created by Xiao Yang on 15/4/12.
//  Copyright (c) 2015年 Young. All rights reserved.
//

#ifndef __CVAssessments__motionTrack__
#define __CVAssessments__motionTrack__

#ifndef __CVAssessments__opencv__
#define __CVAssessments__opencv__
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#endif


#ifndef __CVAssessments__iostream__
#define __CVAssessments__iostream__

#include <iostream>

using namespace cv;
using namespace std;

#endif

// the variable with prefix binary_ is the image consist of 0 and 255
// meanwhile prefix mask_ is the image consist of 0 and 1
class motionTracker
{
private:
    int counter;
    
protected:
    const string* winName;
    Mat reference;
    VideoCapture* video;
    vector<string> targets;
    
    int SENSITIVITY_VALUE = 12;
    const int SEC_SENSITIVITY_VALUE = 38;
    int UNREALITIVE_THRESHOLD = 20;
    int BLUR_SIZE = 10;
    
    
public:
    enum{ BINARY = 0, BINARY_INV = 1 };
    int x;
    int y;
    void resetVideo();
    void setVideoAndWinName(VideoCapture& _video, const string& win_name);
    void getReferenceImg(void);
    void getDifference(Mat& binary_dst, const Mat& gray_frame1, const Mat& gray_frame2, const int thresholding);
    bool isOverlap(const Mat& mask_frame1, const Mat& mask_frame2);
    void getBackground(const Mat& mask, const Mat& head_frame, const Mat& unoverlapped_frame);
    void getTarget(const Mat& frame);
    void getContours(const Mat& mask);
    void drawCross(const Mat& frame, int x, int y);
    void outputPosition() const;
    Mat binary2mask(const Mat& binary_frame, const int type=BINARY);
};
string intToString(int number);


#endif /* defined(__CVAssessments__motionTrack__) */
