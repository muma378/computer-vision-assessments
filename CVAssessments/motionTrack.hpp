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
    int x;
    int y;
    void setVideoAndWinName(VideoCapture& _video, const string& win_name);
    void getReferenceImg(void);
    void getDifference(Mat& thres_image, const Mat& frame1, const Mat& frame2, const int T);
    bool isOverlap(const Mat& adi1, const Mat& adi2);
    void getBackground(const Mat& mask, const Mat& frame1, const Mat& frame2);
    void getTarget(const Mat& frame);
    void getContours(const Mat& mask);
    void drawCross(const Mat& frame, int x, int y);
    void outputPosition() const;
};
string intToString(int number);


#endif /* defined(__CVAssessments__motionTrack__) */
