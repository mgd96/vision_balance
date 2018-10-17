// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SEGMENTOR_THREAD_HPP__
#define __SEGMENTOR_THREAD_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>

#include <yarp/sig/all.h>

#include "cv.h"
//#include "highgui.h" // to show windows

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "TravisLib.hpp"




#include <yarp/dev/all.h>
#include <yarp/dev/IOpenNI2DeviceDriver.h>

#include <yarp/sig/all.h>

#include <cv.h>
//#include <highgui.h> // to show windows

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "TravisLib.hpp"


#define DEFAULT_ALGORITHM "blueMinusRed"
#define pi  3.141592654
#define DEFAULT_LOCATE "centroid"
#define DEFAULT_MAX_NUM_BLOBS 1
#define DEFAULT_MORPH_CLOSING 2
#define DEFAULT_OUT_FEATURES "locX locY angle area"  // it's a bottle!!
#define DEFAULT_OUT_FEATURES_FORMAT 0
#define DEFAULT_OUT_IMAGE 1
#define DEFAULT_RATE_MS 20
#define DEFAULT_SEE_BOUNDING 3
#define DEFAULT_THRESHOLD 55
#define DEFAULT_KINECT_DEVICE "OpenNI2DeviceServer"
#define DEFAULT_KINECT_LOCAL "/colorRegionDetection"
#define DEFAULT_KINECT_REMOTE "/OpenNI2"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

using namespace cv;
using namespace std;

namespace roboticslab
{

class SegmentorThread : public RateThread {
private:
    BufferedPort<ImageOf<PixelRgb> > *pInImg;
    BufferedPort<ImageOf<PixelRgb> > *pOutImg;  // for testing
    Port *pOutPort;
    //
    ConstString algorithm;
    ConstString locate;
    int maxNumBlobs;
    double morphClosing;
    Bottle outFeatures;
    int outFeaturesFormat;
    int outImage;
    int seeBounding;
    int threshold;

    //global
    int boardHeight = 6;
    int boardWidth = 9;
    int times = 0;
    double c1;
    double c2;
    double c1_2;
    double c2_2;
    double angle;
    double angle_2;

    Size cbSize = Size(boardHeight, boardWidth);

    string filename = "//home//teo//Descargas//calib//DataCam.yml";

    bool doneYet = false;

    //default image size
    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;

    //function prototypes
    //void generate_boardPoints();
    //finglobal

    /*
    int aux;
    int aux2;
    int times_lines;
    vector<Point2f> pointstotrack;
    Point original1;
    Point original2;
    Point  ref;
    vector<Point2f> prev_points;
    vector<Vec4i> lines;

    Mat image, frame;
    Mat image_grey;
    Mat prev_image_grey;
    Mat image_color;
    Mat lines_image;
    Mat filtered_img; //dst

    float PTT1X;
    float PTT1Y;
    float PTT2X;
    float PTT2Y;
    float c1;
    float c2;
    float H;*/


    yarp::dev::IOpenNI2DeviceDriver *kinect;

    Mat inCvMat;
    Mat outCvMat;


    //
    float area, hue_peak, hue_mode, hue_mean, hue_stddev,
    saturation_peak, saturation_mean, saturation_stddev,
    value_peak, value_mode, value_mean, value_stddev, locX, locY,
    rectangularity, axisFirst, axisSecond,
    aspectRatio, solidity, massCenterlocX, massCenterlocY, arc, radius;

public:
    SegmentorThread() : RateThread(DEFAULT_RATE_MS),
        area(-1), hue_peak(-1), hue_mode(-1), hue_mean(-1), hue_stddev(-1),
        saturation_peak(-1), saturation_mean(-1), saturation_stddev(-1),
        value_peak(-1), value_mode(-1), value_mean(-1), value_stddev(-1),
        locX(-1), locY(-1),
        rectangularity(-1), axisFirst(-1), axisSecond(-1),
        aspectRatio(-1), solidity(-1), massCenterlocX(-1), massCenterlocY(-1),
        arc(-1), radius(-1){

    }
    void setIKinectDeviceDriver(yarp::dev::IOpenNI2DeviceDriver * _kinect);
    void setInImg(BufferedPort<ImageOf<PixelRgb> > * _pInImg);
    void setOutImg(BufferedPort<ImageOf<PixelRgb> > * _pOutImg);
    void setOutPort(Port *_pOutPort);
    void init(ResourceFinder &rf);
    void run();  // The periodical function
    Mat filters_application(Mat);
    int lines_detector(int &,vector<Vec4i>&, Mat&, Mat&, vector<Point2f>&, vector<Point2f>&, vector<Point2f>&, Point &, Point &, int &);
};

}  // namespace roboticslab

#endif  // __SEGMENTOR_THREAD_HPP__

