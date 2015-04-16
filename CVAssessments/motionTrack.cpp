//
//  motionTrack.cpp
//  CVAssessments
//
//  Created by Xiao Yang on 15/4/12.
//  Copyright (c) 2015年 Young. All rights reserved.
//

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
#include "motionTrack.hpp"


//int to string helper function
string intToString(int number){
    
    //this function has a number input and string output
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void motionTracker::resetVideo()
{
    video->set(CAP_PROP_POS_FRAMES, 0);

}

void motionTracker::setVideoAndWinName(VideoCapture& _video, const string& _winName)
{
    if( !_video.isOpened() || _winName.empty())
        return;
    
    video = &_video;
    winName = &_winName;
//    video->set(CAP_PROP_POS_FRAMES, 5);
}


void motionTracker::getReferenceImg(void)
{
    Mat head_frame, frame;
    Mat gray_head, gray_frame;
    video->read(head_frame);
    if (head_frame.empty()) {
        cout << "\n Can't read the video file ";
        return;
    }
    cvtColor(head_frame, gray_head, COLOR_BGR2GRAY);
    
    Mat head_adi(head_frame.size(), CV_8UC1, Scalar(0));
    Mat adi(head_frame.size(), CV_8UC1, Scalar(0));
    
    video->read(frame);
    cvtColor(frame, gray_frame, COLOR_BGR2GRAY);
    getDifference(head_adi, gray_head, gray_frame, SENSITIVITY_VALUE);
    
    Mat pre_frame(gray_frame.size(), gray_frame.type());
    do{
        gray_frame.copyTo(pre_frame);
        video->read(frame);
        if(frame.empty())
        {
            cout << "didn't found" <<endl;
            return;
            
        }

        cvtColor(frame, gray_frame, COLOR_BGR2GRAY);
        getDifference(adi, gray_frame, pre_frame, SENSITIVITY_VALUE);
        
    }while (!isOverlap(head_adi, adi));
    
    getBackground(head_adi, head_frame, frame);
}


void motionTracker::getDifference(Mat& adi, const Mat& frame1, const Mat& frame2, const int T)
{
    Mat diff_image, thres_image;
    //calculat the diffrent pixels in the 2 frames
    absdiff(frame1, frame2, diff_image);
    
    //filter the pixels which has small change
    threshold(diff_image, thres_image, T, 255, THRESH_BINARY);
//    imshow("diff adi", thres_image);    //TO DELETE
    //filter the pixels which is noise
    blur(thres_image, thres_image, Size(BLUR_SIZE, BLUR_SIZE));
    threshold(thres_image, thres_image, T, 255, THRESH_BINARY);
//    imshow("head adi", thres_image);    //TO DELETE
//    waitKey();
    thres_image.copyTo(adi);
}

bool motionTracker::isOverlap(const Mat& adi1, const Mat& adi2)
{
    Mat bitand_result;
    bitwise_and(adi1, adi2, bitand_result);
//    cout << countNonZero(bitand_result) << endl;
    if(countNonZero(bitand_result) > UNREALITIVE_THRESHOLD)
        return false;
    else
        return true;
}

void motionTracker::getBackground(const Mat& ball_mask, const Mat& frame1, const Mat& frame2)
{
    frame1.copyTo(reference);
    frame2.copyTo(reference, ball_mask);
//    imshow("background", reference);
//    waitKey();
    cvtColor(reference, reference, COLOR_BGR2GRAY);
}

void motionTracker::getTarget(const Mat& frame)
{
    Mat target_mask, gray_frame;
    cvtColor(frame, gray_frame, COLOR_BGR2GRAY);
    getDifference(target_mask, reference, gray_frame, SEC_SENSITIVITY_VALUE);
    getContours(target_mask);
}

void motionTracker::getContours(const Mat& mask)
{
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    Rect objectBoundingRectangle;
    int target[2];
    if(contours.size()>0)
    {
        //the largest contour is found at the end of the contours vector
        //we will simply assume that the biggest contour is the object we are looking for.
        vector< vector<Point> > largestContourVec;
        largestContourVec.push_back(contours.at(contours.size()-1));
        //make a bounding rectangle around the largest contour then find its centroid
        //this will be the object's final estimated position.
        objectBoundingRectangle = boundingRect(largestContourVec.at(0));
        x = objectBoundingRectangle.x+objectBoundingRectangle.width/2;
        y = objectBoundingRectangle.y+objectBoundingRectangle.height/2;
        
        //update the objects positions by changing the 'theObject' array values
        target[0] = x , target[1] = y;
    }
    
    //make some temp x and y variables so we dont have to type out so much
    targets.push_back(intToString(x));
    targets.push_back(intToString(y));
    
}

void motionTracker::outputPosition() const
{
    ofstream x_pos("x.csv");
    ofstream y_pos("y.csv");
    
    vector<string>::const_iterator t;
    for (t=targets.begin(); t!=targets.end(); t++) {
        x_pos << *t << ",";
        t++;
        y_pos << *t << ",";
    }
    x_pos.close();
    y_pos.close();

}

void motionTracker::drawCross(const Mat& frame, int x, int y)
{
    //draw some crosshairs on the object
    circle(frame,Point(x,y),20,Scalar(0,255,0),2);
    line(frame,Point(x,y),Point(x,y-15),Scalar(0,255,0),2);
    line(frame,Point(x,y),Point(x,y+15),Scalar(0,255,0),2);
    line(frame,Point(x,y),Point(x-15,y),Scalar(0,255,0),2);
    line(frame,Point(x,y),Point(x+15,y),Scalar(0,255,0),2);
}

Mat motionTracker::binary2mask(const Mat& frame, const int type)
{
    Mat dst;
    if (type == BINARY)
        threshold(frame, dst, 0, 1, THRESH_BINARY);
    else if (type == BINARY_INV)
        threshold(frame, dst, 0, 255, THRESH_BINARY);
    
    return dst;
}