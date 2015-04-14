//
//  kalmanFilterTrack.cpp
//  CVAssessments
//
//  Created by Xiao Yang on 15/4/13.
//  Copyright (c) 2015年 Young. All rights reserved.
//

#include "kalmanFilterTrack.hpp"


void kalmanFilterTracker::skipNFrames()
{
    if (!video->isOpened()) {
        return;
    }
    video->set(CAP_PROP_POS_FRAMES, SKIPPED_FRAMES);
}

void kalmanFilterTracker::initKalmanFilter()
{
    const int state_dim = 4;
    const int measure_dim = 2;
    const int control_dim = 0;
    const int dt = 1;    //may need tune
//    KalmanFilter KF/(state_dim, measure_dim, control_dim);
    KF.init(state_dim, measure_dim, control_dim);

    
//    Mat processNoise(4, 1, CV_32F); //
    Mat measurement = Mat::zeros(2, 1, CV_32F);
//    Mat* measurement;
    
    KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, dt, 0,\
                           0, 1, 0, dt,\
                           0, 0, 1, 0,\
                           0, 0, 0, 1); // F: motion model
    
    setIdentity(KF.measurementMatrix);  // H: observation model
    cout << KF.measurementMatrix;
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5)); //Q:noise of motion model
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-5));    //R: noise of observation model
    setIdentity(KF.errorCovPost, Scalar::all(1));   //P: posteriori error estimate covariance matrix (P(k))
    
    Mat frame;
    video->read(frame);
    measurement = getMeasurement(frame);
    KF.statePost = (Mat_<float>(4, 1) << measurement.at<float>(0), measurement.at<float>(1), 1, 1);
    
    while (video->get(CAP_PROP_POS_FRAMES)<video->get(CAP_PROP_FRAME_COUNT)-1) {
        video->read(frame);
        
        Mat prediction = KF.predict();
        cout << "prediction:" << prediction.at<float>(0) << ", " << prediction.at<float>(1)  << endl;
        measurement = getMeasurement(frame);
        
        Mat estimated = KF.correct(measurement);
        cout << "estimated:" << estimated.at<float>(0) << ", " << estimated.at<float>(1) << endl;
//        cout << "estimated:" << estimated;
        
        drawCross(frame, prediction.at<float>(0), prediction.at<float>(1));
        imshow(*winName, frame);
        waitKey(10);
        
    }
}

Mat kalmanFilterTracker::getMeasurement(const Mat &frame)
{
    getTarget(frame);
    cout << "measurement:" << x << ", " << y << endl;
    Mat measure = (Mat_<float>(2, 1) << x, y);
    return measure;

}

void kalmanFilterTracker::getTarget(const Mat& frame)
{
    
//    const int R = 30;
//    Point predict_point(prediction.at<float>(0), prediction.at<float>(1));
    Mat target_mask, gray_frame;
//    Mat predict_image;
//    Mat predict_mask(frame.size(), CV_8UC1, Scalar::all(0));
    cvtColor(frame, gray_frame, COLOR_BGR2GRAY);
    getDifference(target_mask, reference, gray_frame, SEC_SENSITIVITY_VALUE);
    
//    Mat target_frame;
//    threshold(target_mask, target_mask, 0, 1, THRESH_BINARY);
//    gray_frame.copyTo(target_frame, target_mask);
//    imshow("target frame", target_frame);
//    
//    Mat temp1, temp2;
//    threshold(target_frame, temp1, 60, 1, THRESH_BINARY);
//    threshold(target_frame, temp2, 83, 1, THRESH_BINARY_INV);
//    bitwise_and(temp1, temp2, target_mask);
//    threshold(target_mask, target_mask, 0, 255, THRESH_BINARY);
//    blur(target_mask, target_mask, Size(BLUR_SIZE, BLUR_SIZE));
//    threshold(target_mask, target_mask, 0, 255, THRESH_BINARY);
//    imshow("dual-threshold", target_mask);
    
//    circle(predict_mask, predict_point, R, Scalar::all(1), -1);
//    threshold(predict_mask, predict_image, 0, 255, THRESH_BINARY);
//    imshow("circle", predict_image);
//    bitwise_and(target_mask, predict_mask, target_mask);
    getContours(target_mask);
    
}

const Mat& extendedKalmanFilter::correct(const Mat &measurement)
{
    predictObservation = measurementMatrix * statePost;     // Z'
    innovationCov = measurementMatrix * errorCovPre * measurementMatrix.t() + measurementNoiseCov;  // S
    Mat invInnovationCov;
    invert(innovationCov, invInnovationCov);
    gain = errorCovPre * measurementMatrix.t() * invInnovationCov;  //K
    
    if(gating(measurement, invInnovationCov)){
        statePost = statePre + gain * (measurement - predictObservation);
        errorCovPost = errorCovPre - gain * innovationCov * gain.t();
    }else{
        statePost = statePre;
        errorCovPost = errorCovPre;
    }
    return statePost;
}

bool extendedKalmanFilter::gating(const Mat &measurement, const Mat& invInnovationCov)
{
    const float gate = 9.21;
    Mat distance = (measurement - predictObservation).t() * invInnovationCov * (measurement - predictObservation);
//    return distance.at<float>(0) <= gate;
    return true;
}



