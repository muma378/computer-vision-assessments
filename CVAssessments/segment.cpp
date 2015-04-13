//
//  segment.cpp
//  CVAssessments
//
//  Created by Xiao Yang on 15/4/10.
//  Copyright (c) 2015年 Young. All rights reserved.
//

#include <fstream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include "boost/archive/binary_iarchive.hpp"
#include "boost/serialization/split_free.hpp"
#include "boost/serialization/vector.hpp"
#include "segment.hpp"



const Scalar GREEN = Scalar(0, 255, 0);
const Scalar RED = Scalar(0, 0, 255);
const Scalar BLUE = Scalar(255, 0, 0);

const int BGD_KEY = EVENT_FLAG_CTRLKEY;
const int FGD_KEY = EVENT_FLAG_SHIFTKEY;


BOOST_SERIALIZATION_SPLIT_FREE(Mat)
namespace boost {
    namespace serialization {
        
        /*** Mat ***/
        template<class Archive>
        void save(Archive & ar, const Mat& m, const unsigned int version)
        {
            size_t elemSize = m.elemSize(), elemType = m.type();
            
            ar & m.cols;
            ar & m.rows;
            ar & elemSize;
            ar & elemType; // element type.
            size_t dataSize = m.cols * m.rows * m.elemSize();
            
            //cout << "Writing matrix data rows, cols, elemSize, type, datasize: (" << m.rows << "," << m.cols << "," << m.elemSize() << "," << m.type() << "," << dataSize << ")" << endl;
            
            for (size_t dc = 0; dc < dataSize; ++dc) {
                ar & m.data[dc];
            }
        }
        
        template<class Archive>
        void load(Archive & ar, Mat& m, const unsigned int version)
        {
            int cols, rows;
            size_t elemSize, elemType;
            
            ar & cols;
            ar & rows;
            ar & elemSize;
            ar & elemType;
            
            m.create(rows, cols, elemType);
            size_t dataSize = m.cols * m.rows * elemSize;
            
            //cout << "reading matrix data rows, cols, elemSize, type, datasize: (" << m.rows << "," << m.cols << "," << m.elemSize() << "," << m.type() << "," << dataSize << ")" << endl;
            
            for (size_t dc = 0; dc < dataSize; ++dc) {
                ar & m.data[dc];
            }
        }
        
    }
}


static void getBinMask( const Mat& comMask, Mat& binMask )
{
    if( comMask.empty() || comMask.type()!=CV_8UC1 )
        CV_Error( Error::StsBadArg, "comMask is empty or has incorrect type (not CV_8UC1)" );
    if( binMask.empty() || binMask.rows!=comMask.rows || binMask.cols!=comMask.cols )
        binMask.create( comMask.size(), CV_8UC1 );
    binMask = comMask & 1;
}


void GCApplication::reset()
{
    if( !mask.empty() )
        mask.setTo(Scalar::all(GC_BGD));
    bgdPxls.clear(); fgdPxls.clear();
    
    isInitialized = false;
    rectState = NOT_SET;
    lblsState = NOT_SET;
    iterCount = 0;
}

void GCApplication::setImageAndWinName(const Mat& _image, const string& _winName)
{
    if( _image.empty() || _winName.empty())
        return;
    
    image = &_image;
    winName = &_winName;
    mask.create(image->size(), CV_8UC1);
    reset();
}

void GCApplication::showImage() const
{
    if( image->empty() || winName->empty() )
        return;
    Mat res;
    Mat binMask;
    
    if( !isInitialized )
        image->copyTo(res);
    else{
        getBinMask( mask, binMask );
        image->copyTo( res, binMask );
    }

    vector<Point>::const_iterator it;
    for( it = bgdPxls.begin(); it != bgdPxls.end(); ++it )
        circle(res, *it, radius, BLUE, thickness);
    for( it = fgdPxls.begin(); it != fgdPxls.end(); ++it )
        circle(res, *it, radius, RED, thickness);

    
    if ( rectState == IN_PROCESS || rectState == SET)
        rectangle(res, Point( rect.x, rect.y ), Point( rect.x+rect.width, rect.y+rect.height ), GREEN, 2);
    
    imshow(*winName, res);
    
}

void GCApplication::setRectInMask()
{
    CV_Assert(!mask.empty());
    mask.setTo(GC_BGD);
    rect.x = max(0, rect.x);
    rect.y = max(0, rect.y);
    rect.width = min(rect.width, image->cols-rect.x);
    rect.height = min(rect.height, image->rows-rect.y);
    (mask(rect)).setTo(Scalar(GC_PR_FGD));
}

void GCApplication::setLblsInMask(int flags, Point p)
{
    if(flags & BGD_KEY)
    {
        bgdPxls.push_back(p);
        circle(mask, p, radius, GC_BGD, thickness);
    }
    if(flags & FGD_KEY)
    {
        fgdPxls.push_back(p);
        circle(mask, p, radius, GC_FGD, thickness);
    }

}

void GCApplication::mouseClick(int event, int x, int y, int flags, void *param)
{
    switch (event) {
        case EVENT_LBUTTONDOWN:
        {
            bool isb = (flags & BGD_KEY) != 0,
                 isf = (flags & FGD_KEY) != 0;
            if( rectState == NOT_SET && !isb && !isf)
            {
                rectState = IN_PROCESS;
                rect = Rect(x, y, 1, 1);
            }
            if ((isb || isf) && rectState == SET) {
                lblsState = IN_PROCESS;
            }
        
            break;
        }
        case EVENT_LBUTTONUP:
        {
            if(rectState == IN_PROCESS)
            {
                rect = Rect(Point(rect.x, rect.y), Point(x, y));
                rectState = SET;
                setRectInMask();
                CV_Assert(bgdPxls.empty() && fgdPxls.empty());
                showImage();
            }
            if(lblsState == IN_PROCESS)
            {
                setLblsInMask(flags, Point(x, y));
                lblsState = SET;
                showImage();
            }
            break;
        }
        case EVENT_MOUSEMOVE:
        {
            if (rectState == IN_PROCESS)
            {
                rect = Rect( Point(rect.x, rect.y), Point(x,y) );
                CV_Assert( bgdPxls.empty() && fgdPxls.empty());
                showImage();
            }
            else if( lblsState == IN_PROCESS )
            {
                setLblsInMask(flags, Point(x,y));
                showImage();
            }
            break;
        }
    }
}
    
int GCApplication::nextIter()
{
    if( isInitialized )
        grabCut( *image, mask, rect, bgdModel, fgdModel, 1 );
    else
    {
        if( rectState != SET )
            return iterCount;
            
        if( lblsState == SET )
            grabCut( *image, mask, rect, bgdModel, fgdModel, 1, GC_INIT_WITH_MASK );
        else
            grabCut( *image, mask, rect, bgdModel, fgdModel, 1, GC_INIT_WITH_RECT );
            
        isInitialized = true;
    }
    iterCount++;
        
    bgdPxls.clear(); fgdPxls.clear();
        
    return iterCount;
}

Mat GCApplication::getImage() const
{
    Mat binMask, result;
    getBinMask( mask, binMask );
    image->copyTo( result, binMask );
    return result;
}

void GCApplication::saveMask()
{
    if(mask.empty())
    {
        cout << "\n There isn't proper mask can be saved ";
        return;
    }
    ofstream mask_ofile("mask", fstream::binary);
    boost::archive::binary_oarchive oa(mask_ofile);
    oa << mask;
        
}

void GCApplication::loadMask()
{
    reset();
    ifstream mask_ifile("mask");
    boost::archive::binary_iarchive ia(mask_ifile);
    ia >> mask;
    if(mask.empty())
    {
        cout << "\n Can't load the mask named \"mask\" ";
        return;
    }
    rectState = SET;
    lblsState = SET;
}


