//
//  segment.hpp
//  CVAssessments
//
//  Created by Xiao Yang on 15/4/10.
//  Copyright (c) 2015年 Young. All rights reserved.
//

#ifndef CVAssessments_segment_hpp
#define CVAssessments_segment_hpp

#include <iostream>


using namespace cv;
using namespace std;


class GCApplication
{
public:
    enum{ NOT_SET = 0, IN_PROCESS = 1, SET = 2 };
    static const int radius = 2;
    static const int thickness = -1;
    
    void reset();
    void setImageAndWinName (const Mat& _image, const string& _winName);
    void showImage() const;
    void mouseClick( int event, int x, int y, int flags, void* param );
    int nextIter();
    int getIterCount() const {return iterCount;}
    Mat getImage() const;
    void saveMask();
    void loadMask();
    
private:
    void setRectInMask();
    void setLblsInMask(int flags, Point p);
    
    const string* winName;
    const Mat* image;
    Mat mask;
    Mat bgdModel, fgdModel;
    uchar rectState, lblsState;
    bool isInitialized;
    
    Rect rect;
    vector<Point>fgdPxls, bgdPxls;
    int iterCount;
};


#endif
