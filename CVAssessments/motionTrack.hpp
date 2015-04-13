//
//  motionTrack.h
//  CVAssessments
//
//  Created by Xiao Yang on 15/4/12.
//  Copyright (c) 2015年 Young. All rights reserved.
//

#ifndef __CVAssessments__motionTrack__
#define __CVAssessments__motionTrack__

#include <iostream>
using namespace std;
using namespace cv;

class motionTracker
{
private:
    const string* winName;
    Mat reference;
    VideoCapture* video;
    vector<string> targets;
    int counter;
    
public:
    int currentFrame;
    void setVideoAndWinName(VideoCapture& _video, const string& win_name);
    void getReferenceImg(void);
    void getDifference(Mat& thres_image, const Mat& frame1, const Mat& frame2, const int T);
    bool isOverlap(const Mat& adi1, const Mat& adi2);
    void getBackground(const Mat& mask, const Mat& frame1, const Mat& frame2);
    void getTarget(const Mat& frame);
    void outputPosition() const;
};


#endif /* defined(__CVAssessments__motionTrack__) */
