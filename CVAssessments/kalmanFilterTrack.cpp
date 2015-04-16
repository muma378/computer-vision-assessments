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
    
    // based on the model: d(t) = d(t-1) + (d(t-1) - d(t-2))
    KF.transitionMatrix = (Mat_<float>(4, 4) << 2, 0, -1, 0,\
                           0, 2, 0, -1,\
                           1, 0, 0, 0,\
                           0, 1, 0, 0); // F: motion model
    
    setIdentity(KF.measurementMatrix);  // H: observation model
    setIdentity(KF.processNoiseCov, Scalar::all(0.9)); //Q:noise of motion model
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-3));    //R: noise of observation model
    setIdentity(KF.errorCovPost, Scalar::all(1));   //P: posteriori error estimate covariance matrix (P(k))
    
    Mat frame;
    video->read(frame);
    getDominantColor(frame);    //TODO
    measurement = getMeasurement(frame);
    KF.statePost = (Mat_<float>(4, 1) << measurement.at<float>(0), measurement.at<float>(1), measurement.at<float>(0), measurement.at<float>(1));
    
    while (video->get(CAP_PROP_POS_FRAMES)<video->get(CAP_PROP_FRAME_COUNT)-1) {
        video->read(frame);
        Mat prediction = KF.predict();
        cout << "prediction:" << prediction.at<float>(0) << ", " << prediction.at<float>(1)  << endl;
        measurement = getMeasurement(frame);
        
        Mat estimated = KF.correct(measurement);
        cout << "estimated:" << estimated.at<float>(0) << ", " << estimated.at<float>(1) << endl;
        
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
    
    Mat target_mask, gray_frame;
    cvtColor(frame, gray_frame, COLOR_BGR2GRAY);
    getDifference(target_mask, reference, gray_frame, SEC_SENSITIVITY_VALUE);
    Mat filtered_frame;
    frame.copyTo(filtered_frame, target_mask);
    imshow("filtered frame", filtered_frame);
    waitKey(10);
    thresholdWithDomC(filtered_frame, target_mask);
    getContours(target_mask);
}

void kalmanFilterTracker::thresholdWithDomC(const Mat& frame, Mat& mask)
{
//    mask.create(frame.rows, frame.cols, CV_8UC1);
    
    float b = dominantColor[0], g = dominantColor[1], r = dominantColor[2];
    Vec3b pixel;
    for(int x=0;x<frame.rows;x++){
        for (int y=0; y<frame.cols; y++) {
            pixel = frame.at<Vec3b>(x, y);
            if (abs(pixel[0]-b) < THRESH_B && abs(pixel[1]-g) < THRESH_G && abs(pixel[2]-r) < THRESH_R)
            {
                mask.at<uchar>(x, y) = 1;
            }else
                mask.at<uchar>(x, y) = 0;
        }
    }
    
    Mat binary_frame = binary2mask(mask,BINARY_INV);
    imshow("binary_frame", binary_frame);
    GaussianBlur(binary_frame, binary_frame, Size(3, 3), 1.5);
    mask = binary2mask(binary_frame);

    
}


int kalmanFilterTracker::getMostOftenLabel(Mat& labels)
{
    
    int counter[CLUSTER_NUM], i;
    for (i=0; i<CLUSTER_NUM; i++) {
        counter[i] = 0;
    }
    for (i=0;i<labels.rows;i++) {
        counter[labels.at<int>(i, 0)]++;
    }
    int labels_count = 0, max=0, dominant_label=0;
    for (i=0; i<CLUSTER_NUM; i++) {
        labels_count += counter[i];
        if (MAX(max, counter[i]) != max) {
            max = counter[i];
            dominant_label = i;
        }
    }
    if (labels_count != labels.rows) {
        cout << "getMostOftenLabel: count failed" << endl;
        exit(0);
    }
    
    return dominant_label;
}


void kalmanFilterTracker::getDominantColor(const Mat& color_frame)
{
    Mat binary_dst, mask;
    Mat frame;
    cvtColor(color_frame, frame, COLOR_BGR2GRAY);
    getDifference(binary_dst, frame, reference, SENSITIVITY_VALUE);
    mask = binary2mask(binary_dst);
    
    Mat target;
    color_frame.copyTo(target, mask);

    Mat features, labels, centers;
    getNonzeroFeatures(target, features);
    kmeans(features, CLUSTER_NUM, labels, TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),
           3, KMEANS_PP_CENTERS, centers);

    float* dominant_center = centers.ptr<float>(getMostOftenLabel(labels));
    dominantColor = Scalar(dominant_center[0], dominant_center[1], dominant_center[2]);
//    cout << dominantColor;
//    cout << centers;
//    drawCross(target, centers.at<float>(dominant_label, 3), centers.at<float>(dominant_label, 4));
//    imshow("target color", target);
//    waitKey();

}


void kalmanFilterTracker::getNonzeroFeatures(const Mat& frame, Mat& features)
{
    int feature_num = 5, pixels_num = 0;
    Mat I;
    frame.copyTo(I);
    MatIterator_<Vec3b> it, end;
    for( it = I.begin<Vec3b>(), end = I.end<Vec3b>(); it != end; ++it)
    {
        if ((*it)[0] == 0 && (*it)[1] == 0 && (*it)[2] == 0) {
            continue;
        }
        pixels_num++;
    }
    
    Vec3b pixel;
    features.create(pixels_num, feature_num, CV_32FC1);
    int i = 0;
    for(int x=0;x<frame.rows;x++){
        for (int y=0; y<frame.cols; y++) {
            pixel = frame.at<Vec3b>(x, y);
            if (pixel(0) == 0 && pixel(1) == 0 && pixel(2) == 0) {
                continue;
            }
            if (i > pixels_num){
                cout << "getNonzeroFeatures: out of boundray" << endl;
                exit(1);
            }
            features.at<float>(i, 0) = pixel[0];
            features.at<float>(i, 1) = pixel[1];
            features.at<float>(i, 2) = pixel[2];
            features.at<float>(i, 3) = x;
            features.at<float>(i, 4) = y;
            i++;
        }
    }

}


const Mat& extendedKalmanFilter::correct(const Mat &measurement)
{
    predictObservation = measurementMatrix * statePost;     // Z'
    innovationCov = measurementMatrix * errorCovPre * measurementMatrix.t() + measurementNoiseCov;  // S
    Mat invInnovationCov;
    invert(innovationCov, invInnovationCov);
    gain = errorCovPre * measurementMatrix.t() * invInnovationCov;  //K
    
    if(gating(measurement, invInnovationCov) || gating_counter == 10){
        statePost = statePre + gain * (measurement - predictObservation);
        errorCovPost = errorCovPre - gain * innovationCov * gain.t();
        gating_counter = 0;
    }else{
        cout << "*************gating****************" << endl;
        statePost = statePre;
        errorCovPost = errorCovPre;
        gating_counter++;
    }
    return statePost;
}

bool extendedKalmanFilter::gating(const Mat &measurement, const Mat& invInnovationCov)
{
    const float gate = 9.21;
    Mat distance = (measurement - predictObservation).t() * invInnovationCov * (measurement - predictObservation);
    return distance.at<float>(0) <= gate;
//    return true;
}





