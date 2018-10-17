// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SEGMENTOR_THREAD_HPP__
#define __SEGMENTOR_THREAD_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>


//includes
#include <stdio.h>
#include <string>

#include<iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "math.h";

using namespace std;
using namespace cv;



#include <yarp/dev/all.h>
#include <yarp/dev/IOpenNI2DeviceDriver.h>

#include <yarp/sig/all.h>

#include <cv.h>
//#include <highgui.h> // to show windows

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



#include "TravisLib.hpp"

// thanks! https://web.stanford.edu/~qianyizh/projects/scenedata.html
#define DEFAULT_FX_D          525.0  // 640x480
#define DEFAULT_FY_D          525.0  //
#define DEFAULT_CX_D          319.5  //
#define DEFAULT_CY_D          239.5  //
#define DEFAULT_FX_RGB        525.0  //
#define DEFAULT_FY_RGB        525.0  //
#define DEFAULT_CX_RGB        319.5  //
#define DEFAULT_CY_RGB        239.5  //
#define pi  3.141592654

#define DEFAULT_ALGORITHM "blueMinusRed"
#define DEFAULT_LOCATE "centroid"
#define DEFAULT_MAX_NUM_BLOBS 2
#define DEFAULT_MORPH_CLOSING 2
#define DEFAULT_MORPH_OPENING 0
#define DEFAULT_OUT_FEATURES "mmX mmY mmZ"  // it's a bottle!!
#define DEFAULT_OUT_FEATURES_FORMAT 0  // 0=bottled,1=minimal
#define DEFAULT_OUT_IMAGE 1
#define DEFAULT_RATE_MS 20
#define DEFAULT_SEE_BOUNDING 3
#define DEFAULT_THRESHOLD 55

#define DEFAULT_FX_D          525.0  // 640x480

#define DEFAULT_FY_D          525.0  //

#define DEFAULT_CX_D          319.5  //

#define DEFAULT_CY_D          239.5  //

#define DEFAULT_FX_RGB        525.0  //

#define DEFAULT_FY_RGB        525.0  //

#define DEFAULT_CX_RGB        319.5  //

#define DEFAULT_CY_RGB        239.5  //



namespace roboticslab
{

/**
 * @ingroup colorRegionDetection
 *
 * @brief Implements colorRegionDetection callback on Bottle.
 */
class DataProcessor : public yarp::os::PortReader {
    virtual bool read(yarp::os::ConnectionReader& connection) {
        yarp::os::Bottle b;
        b.read(connection);
        // process data in b
        printf("Got %s\n", b.toString().c_str());
        if(waitForFirst) {
            xKeep = b.get(0).asInt();
            yKeep = b.get(1).asInt();
            waitForFirst = false;
        } else {
            if((b.get(0).asInt()<xKeep)||(b.get(1).asInt()<yKeep)){
                x = 0;
                y = 0;
                w = 0;
                h = 0;
            } else {
                x = xKeep;
                y = yKeep;
                w = b.get(0).asInt() - x;
                h = b.get(1).asInt() - y;
            }
            waitForFirst = true;
        }
        return true;

    }
public:
    bool reset() {
        waitForFirst = true;
        x = 0;
        y = 0;
        w = 0;
        h = 0;
        xKeep = 0;
        yKeep = 0;
    }
    int xKeep, yKeep;
    int x, y, w, h;
    bool waitForFirst;
};

/**
 * @ingroup colorRegionDetection
 *
 * @brief Implements colorRegionDetection RateThread.
 */
class SegmentorThread : public yarp::os::RateThread {
private:
    yarp::dev::IOpenNI2DeviceDriver *kinect;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *pOutImg;  // for testing
    yarp::os::Port *pOutPort;
    //
    yarp::os::ConstString algorithm;
    yarp::os::ConstString locate;
    int maxNumBlobs;
    double morphClosing;
    double morphOpening;
    int outFeaturesFormat;
    int outImage;
    int seeBounding;
    int threshold;



    //global
    int option=0;
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

    string filename = "//home//teo//Descargas//calib//teo_params_gd.yml";
    bool doneYet = false;

    //default image size
    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;


    Mat inCvMat;
    Mat outCvMat;



    // end global
    double fx_d,fy_d,cx_d,cy_d,fx_rgb,fy_rgb,cx_rgb,cy_rgb;
    //
    yarp::os::Bottle outFeatures;
    //
    int cropSelector;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* outCropSelectorImg;
    yarp::os::Port* inCropSelectorPort;
    DataProcessor processor;

public:
    SegmentorThread() : RateThread(DEFAULT_RATE_MS) {}

    void setIKinectDeviceDriver(yarp::dev::IOpenNI2DeviceDriver * _kinect);
    void setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg);
    void setOutPort(yarp::os::Port *_pOutPort);
    void init(yarp::os::ResourceFinder &rf);
    void run();  // The periodical function

    void setCropSelector(int cropSelector) { this->cropSelector = cropSelector; }
    void setOutCropSelectorImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* outCropSelectorImg) { this->outCropSelectorImg = outCropSelectorImg; }
    void setInCropSelectorPort(yarp::os::Port* inCropSelectorPort) { this->inCropSelectorPort = inCropSelectorPort; }
    
};

}  // namespace roboticslab

#endif  // __SEGMENTOR_THREAD_HPP__

