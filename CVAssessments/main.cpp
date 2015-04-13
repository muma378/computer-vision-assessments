#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "segment.hpp"
#include "motionTrack.hpp"

#include <iostream>

using namespace std;
using namespace cv;

static void help()
{
    cout << "\nThis program demonstrates GrabCut segmentation -- select an object in a region\n"
            "and then grabcut will attempt to segment it out.\n"
            "Call:\n"
            "./grabcut <image_name>\n"
        "\nSelect a rectangular area around the object you want to segment\n" <<
        "\nHot keys: \n"
        "\tESC - quit the program\n"
        "\tr - restore the original image\n"
        "\tn - next iteration\n"
        "\n"
        "\tleft mouse button - set rectangle\n"
        "\n"
        "\tCTRL+left mouse button - set GC_BGD pixels\n"
        "\tSHIFT+left mouse button - set GC_FGD pixels\n" << endl;
}

GCApplication gcapp;

static void on_mouse( int event, int x, int y, int flags, void* param )
{
    gcapp.mouseClick( event, x, y, flags, param );
}


int segment( int argc, char** argv )
{
    if( argc == 2 )
    {
        help();
        return 1;
    }
    string filename = "SWAN.jpg";
    if( filename.empty() )
    {
        cout << "\nDurn, couldn't read in " << argv[1] << endl;
        return 1;
    }
    Mat image = imread( filename, 1 );
    if( image.empty() )
    {
        cout << "\n Durn, couldn't read image filename " << filename << endl;
        return 1;
    }

    help();

    const string winName = "image";
    namedWindow( winName, WINDOW_AUTOSIZE );
    setMouseCallback( winName, on_mouse, 0 );

    gcapp.setImageAndWinName( image, winName );
    gcapp.showImage();

    for(;;)
    {
        int c = waitKey(0);
        switch( (char) c )
        {
        case '\x1b':
            {
            cout << "Exiting ..." << endl;
            goto exit_main;
            }
        case 'r':
            {
            cout << endl;
            gcapp.reset();
            gcapp.showImage();
            break;
            }
        case 'n':
            {
            int iterCount = gcapp.getIterCount();
            cout << "<" << iterCount << "... ";
            int newIterCount = gcapp.nextIter();
            if( newIterCount > iterCount )
            {
                gcapp.showImage();
                cout << iterCount << ">" << endl;
            }
            else
                cout << "rect must be determined>" << endl;
            break;
            }
        case 's':
            {
                gcapp.saveMask();
                cout << "\n The mask has been saved successfully ";
                break;
            }
        case 'l':
            {
                gcapp.loadMask();
                gcapp.nextIter();
                gcapp.showImage();
                break;
            }
        }
        
    }

exit_main:
    vector<int> compression_params;
    compression_params.push_back(IMWRITE_JPEG_OPTIMIZE);
    compression_params.push_back(9);
    imwrite("Segmentation.jpg", gcapp.getImage(), compression_params);
    destroyWindow( winName );
    return 0;
}

motionTracker mt;

int main(int argc, char** argv)
{
    string file = "/Users/imac/Desktop/Computer Vision/videos/ball.mp4";
    VideoCapture capture(file);
    Mat frame;
    const string winName = "task2";
    namedWindow( winName, WINDOW_AUTOSIZE );
    mt.setVideoAndWinName(capture, winName);
    mt.getReferenceImg();

    capture.release();
    capture.open(file);
    while (capture.get(CAP_PROP_POS_FRAMES)<capture.get(CAP_PROP_FRAME_COUNT)-1) {
        capture.read(frame);
        mt.getTarget(frame);
        imshow(winName, frame);
        waitKey(10);
    }
    mt.outputPosition();
    capture.release();

    
}