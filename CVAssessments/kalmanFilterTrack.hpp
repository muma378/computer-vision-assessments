//
//  kalmanFilterTrack.h
//  CVAssessments
//
//  Created by Xiao Yang on 15/4/13.
//  Copyright (c) 2015年 Young. All rights reserved.
//

#ifndef __CVAssessments__kalmanFilterTrack__
#define __CVAssessments__kalmanFilterTrack__

#include "motionTrack.hpp"
#include "opencv2/video/tracking.hpp"


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


//Kalman Filter with valitation gating
class extendedKalmanFilter:public KalmanFilter
{
public:
    Mat predictObservation;
    Mat innovationCov;
    const Mat& correct(const Mat& measurement);
    bool gating(const Mat& measurement, const Mat& invInnovationCov);
    extendedKalmanFilter(){};
};


class kalmanFilterTracker:public motionTracker
{
private:
    extendedKalmanFilter KF;
    
protected:
    int SKIPPED_FRAMES = 5;
    
public:
    kalmanFilterTracker(){
        SENSITIVITY_VALUE = 12;
        UNREALITIVE_THRESHOLD = 20;
        BLUR_SIZE = 10;};
    void skipNFrames(void);
    void initKalmanFilter();
    Mat getMeasurement(const Mat& frame);
    void getTarget(const Mat& frame);
    
};


#endif /* defined(__CVAssessments__kalmanFilterTrack__) */
