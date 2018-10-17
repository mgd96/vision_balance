// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SegmentorThread.hpp"

using namespace cv;
using namespace std;

namespace roboticslab
{

/************************************************************************/

void SegmentorThread::setInImg(BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pInImg)
{
    pInImg = _pInImg;
}

/************************************************************************/

void SegmentorThread::setOutImg(BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg)
{
    pOutImg = _pOutImg;
}

/************************************************************************/
void SegmentorThread::setOutPort(Port * _pOutPort)
{
    pOutPort = _pOutPort;
}

/************************************************************************/
void SegmentorThread::init(ResourceFinder &rf) {

    algorithm = DEFAULT_ALGORITHM;
    locate = DEFAULT_LOCATE;
    maxNumBlobs = DEFAULT_MAX_NUM_BLOBS;
    morphClosing = DEFAULT_MORPH_CLOSING;
    outImage = DEFAULT_OUT_IMAGE;
    outFeatures.fromString(DEFAULT_OUT_FEATURES);  // it's a bottle!!
    outFeaturesFormat = DEFAULT_OUT_FEATURES_FORMAT;
    int rateMs = DEFAULT_RATE_MS;
    seeBounding = DEFAULT_SEE_BOUNDING;
    threshold = DEFAULT_THRESHOLD;

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("SegmentorThread options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--algorithm (redMinusBlue,greenMinusRed...; default: \"%s\")\n",algorithm.c_str());
        printf("\t--locate (centroid,bottom; default: \"%s\")\n",locate.c_str());
        printf("\t--maxNumBlobs (default: \"%d\")\n",maxNumBlobs);
        printf("\t--morphClosing (percentage, 2 or 4 okay; default: \"%f\")\n",morphClosing);
        printf("\t--outFeatures (default: \"(%s)\")\n",outFeatures.toString().c_str());
        printf("\t--outFeaturesFormat (0=bottled,1=minimal; default: \"%d\")\n",outFeaturesFormat);
        printf("\t--outImage (0=rgb,1=bw; default: \"%d\")\n",outImage);
        printf("\t--rateMs (default: \"%d\")\n",rateMs);
        printf("\t--seeBounding (0=none,1=contour,2=box,3=both; default: \"%d\")\n",seeBounding);
        printf("\t--threshold (default: \"%d\")\n",threshold);
    }

    if (rf.check("algorithm")) algorithm = rf.find("algorithm").asString();
    if (rf.check("locate")) locate = rf.find("locate").asString();
    if (rf.check("maxNumBlobs")) maxNumBlobs = rf.find("maxNumBlobs").asInt();
    if (rf.check("morphClosing")) morphClosing = rf.find("morphClosing").asDouble();
    if (rf.check("outFeaturesFormat")) outFeaturesFormat = rf.find("outFeaturesFormat").asInt();
    printf("SegmentorThread using algorithm: %s, locate: %s, maxNumBlobs: %d, morphClosing: %f, outFeaturesFormat: %d.\n",
           algorithm.c_str(),locate.c_str(),maxNumBlobs,morphClosing,outFeaturesFormat);

    if (rf.check("outFeatures")) {
        outFeatures = *(rf.find("outFeatures").asList());  // simple overrride
    }
    printf("SegmentorThread using outFeatures: (%s).\n", outFeatures.toString().c_str());

    if (rf.check("outImage")) outImage = rf.find("outImage").asInt();
    if (rf.check("rateMs")) rateMs = rf.find("rateMs").asInt();
    if (rf.check("threshold")) threshold = rf.find("threshold").asInt();
    if (rf.check("seeBounding")) seeBounding = rf.find("seeBounding").asInt();
    printf("SegmentorThread using outImage: %d, rateMs: %d, seeBounding: %d, threshold: %d.\n",
           outImage, rateMs, seeBounding, threshold);

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    this->setRate(rateMs);
    this->start();

}
/************************************************************************/
void SegmentorThread::setIKinectDeviceDriver(yarp::dev::IOpenNI2DeviceDriver *_kinect) {
    kinect = _kinect;
}
/************************************************************************/
/*
Mat SegmentorThread:: filters_application(Mat frame){


    Mat hsv;
    Mat ycbcr;
    Mat mask_hsv, mask_ycbcr, mask_or, mask_and;
    Mat kernel;
    Mat and_hsv, and_ycbcr;
    Mat filtered_img(frame.size(), frame.type()); //dst

    int low_threshold = 50;
    int high_threshold = low_threshold * 2;
    int kernel_size = 3;

    blur(frame, frame, Size(3, 3));

    cvtColor(frame, hsv, CV_BGR2HSV);
    cvtColor(frame, ycbcr, CV_BGR2YCrCb);

    inRange(hsv, Scalar(0, 0, 60), Scalar(20, 170, 255), mask_hsv);
    inRange(ycbcr, Scalar(0, 133, 77), Scalar(255, 173, 127), mask_ycbcr);

    cvtColor(mask_hsv, mask_hsv, CV_GRAY2BGR);
    cvtColor(mask_ycbcr, mask_ycbcr, CV_GRAY2BGR);

    kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(mask_hsv, mask_hsv, MORPH_CLOSE, kernel);
    morphologyEx(mask_ycbcr, mask_ycbcr, MORPH_CLOSE, kernel);

    kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(mask_hsv, mask_hsv, kernel);
    dilate(mask_ycbcr, mask_ycbcr, kernel);

    kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(mask_hsv, mask_hsv, kernel);
    erode(mask_ycbcr, mask_ycbcr, kernel);

    bitwise_and(frame, mask_hsv, and_hsv);
    bitwise_and(frame, mask_ycbcr, and_ycbcr);


    bitwise_and(mask_hsv, mask_ycbcr, mask_and);
    bitwise_or(mask_hsv, mask_ycbcr, mask_or);


    Canny(mask_or, filtered_img, low_threshold, high_threshold, kernel_size);


    return filtered_img;
}*/
/************************************************************************/


/*
int SegmentorThread:: lines_detector(int& aux,vector<Vec4i>&lines,Mat & lines_image, Mat & filtered_img, vector<Point2f>&  line_points_start, vector<Point2f>& line_points_end, vector<Point2f>& pointstotrack,Point&original1,Point&original2, int & times_lines){


    HoughLinesP(filtered_img, lines, 1, CV_PI / 180, 50, 50, 10);
    cout<<lines.size()<<endl;

    for (size_t i = 0; i < lines.size(); i++)
    {

        Vec4i l = lines[i];

        //line_points_start[i] = Point(l[0], l[1]);
        //line_points_end[i] = Point(l[2], l[3]);

        //look for another line if the camera lost the previous line(out of range), fix it
        //if (pointstotrack[1].x <= 10 || pointstotrack[2].x >= image.cols - 10 || pointstotrack[1].y <= 10 || pointstotrack[1].y > image.rows - 10 || pointstotrack[2].y <= 10 || pointstotrack[2].y > image.rows - 10){

        /*    if ( (l[1] == l[3])){
        times_lines = 1;
        }
        }


        if((original1.x<5) && (original2.x<5) && (aux2!=0)){
            times_lines=1;
            cout<<original1.x<<"---"<<original2.x<<endl;
        }

        if (times_lines == 1 && (l[1] == l[3])){





            pointstotrack[1]=Point(l[0], l[1]);
            pointstotrack[2]=Point(l[2], l[3]);
            original1 = Point(l[0], l[1]);
            original2 = Point(l[2], l[3]);



            cout<<"busque"<<endl;
            cout<<original1.x<<"---"<<original2.x<<endl;
            aux2=1;
            //line(lines_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);

            times_lines=0;




        }

    }
    return aux2;

}*

/************************************************************************/
void SegmentorThread::run() {




   /* ImageOf<PixelRgb> *inYarpImg = pInImg->read(false);
    if (inYarpImg==NULL) {
        //printf("No img yet...\n");
        return;
    };

    yarp::sig::ImageOf<yarp::sig::PixelMono16> depth = kinect->getDepthFrame();
    if (depth.height()<10) {
        printf("No depth yet...\n");
        return;
    };

    IplImage *inIplImage = cvCreateImage(cvSize(inYarpImg->width(), inYarpImg->height()),
                                         IPL_DEPTH_8U, 3 );




    cvCvtColor((IplImage*)inYarpImg->getIplImage(), inIplImage, CV_RGB2BGR);
    inCvMat = cvarrToMat(inIplImage);




    inCvMat.copyTo(image);
    original1=Point(image.rows/2,image.cols/2);
    circle(image, original1, 6, Scalar(231, 37, 18), -1, 8);
    double mmZ_tmp=depth.pixel(original1.x,original1.y);

    cout<<"x:"<<original1.x<<"y:"<<original1.y<<"z:"<<mmZ_tmp<<endl;


    outCvMat=image;


    IplImage outIplImage = outCvMat;
    //cvCvtColor(&outIplImage,&outIplImage, CV_BGR2RGB);     //CV_BGR2RGB

    ImageOf<PixelRgb> outYarpImg;
    outYarpImg.wrapIplImage(&outIplImage);

    pOutImg->prepare() = outYarpImg;
    pOutImg->write();

    cvReleaseImage( &inIplImage );  // release the memory for the image
    outCvMat.release();


*/







// Main -------------------------------------------------------------------------------------------


    //set up a FileStorage object to read camera params from file
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    // read camera matrix and distortion coefficients from file
    Mat intrinsics, distortion;
    fs["Camera_Matrix"] >> intrinsics;
    fs["Distortion_Coefficients"] >> distortion;
    // close the input file
    fs.release();




    //set up matrices for storage
    Mat webcamImage, gray, one;
    Mat rvec = Mat(Size(3, 1), CV_64F);
    Mat tvec = Mat(Size(3, 1), CV_64F);

    //setup vectors to hold the chessboard corners in the chessboard coordinate system and in the image
    vector<Point2d> imagePoints, imageFramePoints, imageOrigin;
    vector<Point3d> boardPoints, framePoints;


    //generate vectors for the points on the chessboard
    for (int i = 0; i<boardWidth; i++)
    {
        for (int j = 0; j<boardHeight; j++)
        {
            boardPoints.push_back(Point3d(double(i), double(j), 0.0));
        }
    }
    //generate points in the reference frame
    framePoints.push_back(Point3d(0.0, 0.0, 0.0));
    framePoints.push_back(Point3d(5.0, 0.0, 0.0));
    framePoints.push_back(Point3d(0.0, 5.0, 0.0));

    framePoints.push_back(Point3d(0.0, 0.0, 5.0));
    framePoints.push_back(Point3d(1.0,1.0,0.0));
    framePoints.push_back(Point3d(2.0, 2.0, 0.0));
    framePoints.push_back(Point3d(3.0, 3.0, 0.0));





    yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg = kinect->getImageFrame();
    if (inYarpImg.height()<10) {
        //printf("No img yet...\n");
        return;
    };
    yarp::sig::ImageOf<yarp::sig::PixelMono16> depth = kinect->getDepthFrame();
    if (depth.height()<10) {
        //printf("No depth yet...\n");
        return;
    };

    // {yarp ImageOf Rgb -> openCv Mat Bgr}
    IplImage *inIplImage = cvCreateImage(cvSize(inYarpImg.width(), inYarpImg.height()),
                                         IPL_DEPTH_8U, 3 );
    cvCvtColor((IplImage*)inYarpImg.getIplImage(), inIplImage, CV_RGB2BGR);
    cv::Mat inCvMat( cv::cvarrToMat(inIplImage) );




    inCvMat.copyTo(webcamImage);






        //make a gray copy of the webcam image
        cvtColor(webcamImage, gray, COLOR_BGR2GRAY);


        //detect chessboard corners
        bool found = findChessboardCorners(gray, cbSize, imagePoints, CALIB_CB_FAST_CHECK);
        //drawChessboardCorners(webcamImage, cbSize, Mat(imagePoints), found);



        //find camera orientation if the chessboard corners have been found
        if (found)
        {

            //find the camera extrinsic parameters
            solvePnP(Mat(boardPoints), Mat(imagePoints), intrinsics, distortion, rvec, tvec, false);

            //project the reference frame onto the image
            projectPoints(framePoints, rvec, tvec, intrinsics, distortion, imageFramePoints);


            //DRAWING
            //draw the reference frame on the image
            circle(webcamImage, imagePoints[0], 4, CV_RGB(255, 0, 0));

            Point one, two, three;
            one.x = 10; one.y = 10;
            two.x = 60; two.y = 10;
            three.x = 10; three.y = 60;

            line(webcamImage, one, two, CV_RGB(255, 0, 0));
            line(webcamImage, one, three, CV_RGB(0, 255, 0));


            line(webcamImage, imageFramePoints[0], imageFramePoints[1], CV_RGB(255, 0, 0), 2);
            line(webcamImage, imageFramePoints[0], imageFramePoints[2], CV_RGB(0, 255, 0), 2);
            line(webcamImage, imageFramePoints[0], imageFramePoints[3], CV_RGB(0, 0, 255), 2);


            /*line(webcamImage, imageFramePoints[0], Point(imageFramePoints[1].x, imageFramePoints[0].y), CV_RGB(87, 35, 100), 2);
            line(webcamImage, imageFramePoints[0], Point(imageFramePoints[0].x, imageFramePoints[2].y), CV_RGB(87, 35, 100), 2);*/

            circle(webcamImage, Point(imageFramePoints[4].x, imageFramePoints[4].y), 4, CV_RGB(255, 0, 0));
            circle(webcamImage, Point(imageFramePoints[5].x, imageFramePoints[5].y), 4, CV_RGB(255, 0, 0));
            circle(webcamImage, Point(imageFramePoints[6].x, imageFramePoints[6].y), 4, CV_RGB(255, 0, 0));




            /*c1 = imageFramePoints[1].x - imageFramePoints[0].x;
            c2 = imageFramePoints[0].y - imageFramePoints[1].y;

            angle = (atan(c2 / c1))*(180 / 3.141592654);*/

            std::ostringstream strs;
            strs << "rot X: " << rotx*(180 / 3.141592654);
            std::string str = strs.str();

            std::ostringstream strs2;
            strs2 << "rot Y: " << roty*(180 / 3.141592654);
            std::string str2 = strs2.str();

            std::ostringstream strs3;
            strs3 << "rot Z: " << rotz*(180 / 3.141592654);
            std::string str3 = strs3.str();



            putText(webcamImage, str, Point(60, 60), FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0), 2, 8, false);
            putText(webcamImage, str2, Point(60, 90), FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 255, 0), 2, 8, false);
            putText(webcamImage, str3, Point(60, 120), FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 0, 255), 2, 8, false);









            //show the pose estimation data
     /*      cout << fixed  << "rvec = ["
                << rvec.at<double>(0, 0) << ", "
                << rvec.at<double>(1, 0) << ", "
                << rvec.at<double>(2, 0) << "] \t" << "tvec = ["
                << tvec.at<double>(0, 0) << ", "
                << tvec.at<double>(1, 0) << ", "
                << tvec.at<double>(2, 0) << "]" << endl;

        }*/

        //show the image on screen
        outCvMat=webcamImage;






        IplImage outIplImage = outCvMat;
        //cvCvtColor(&outIplImage,&outIplImage, CV_BGR2RGB);     //CV_BGR2RGB

        ImageOf<PixelRgb> outYarpImg;
        outYarpImg.wrapIplImage(&outIplImage);

        pOutImg->prepare() = outYarpImg;
        pOutImg->write();

        //show the gray image
        //namedWindow("Gray Image", CV_WINDOW_AUTOSIZE);
        //imshow("Gray Image", gray);

        cvReleaseImage( &inIplImage );  // release the memory for the image
        outCvMat.release();


    }
}


} // namespace roboticslab







/*

    vector<Point2f> line_points_start(100);
    vector<Point2f> line_points_end(100);
    vector<uchar> optical_flow_found_feature;
    vector<float> optical_flow_feature_error;



    ImageOf<PixelRgb> *inYarpImg = pInImg->read(false);
    if (inYarpImg==NULL) {
        //printf("No img yet...\n");
        return;
    };

    IplImage *inIplImage = cvCreateImage(cvSize(inYarpImg->width(), inYarpImg->height()),
                                         IPL_DEPTH_8U, 3 );







    TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
    Size subPixWinSize(10, 10), winSize(31, 31);




    cvCvtColor((IplImage*)inYarpImg->getIplImage(), inIplImage, CV_RGB2BGR);
    inCvMat = cvarrToMat(inIplImage);




    inCvMat.copyTo(image);



    filtered_img = filters_application(image);


    cvtColor(filtered_img, lines_image, CV_GRAY2BGR);



    aux2= lines_detector(aux,lines,lines_image, filtered_img, line_points_start, line_points_end, pointstotrack, original1, original2, times_lines);
    cout<<"aux2"<<aux2<<endl;



    if (image_grey.rows == 0) {

        image_grey = Mat(inCvMat.size(), CV_8UC1);
        prev_image_grey = Mat(inCvMat.size(), CV_8UC1);
        prev_points = pointstotrack;


    }


    cvtColor(image, image_grey, COLOR_BGR2GRAY);

    image_color = image;



    if (prev_points.size() != 0){
        //aux = 0;
        calcOpticalFlowPyrLK(prev_image_grey, image_grey, prev_points, pointstotrack, optical_flow_found_feature, optical_flow_feature_error, winSize, 3, termcrit, 0, 0.001);

    }

    ref=Point(pointstotrack[2].x,pointstotrack[1].y);

    circle(image_color, pointstotrack[1], 6, Scalar(231, 37, 18), -1, 8);
    circle(image_color, pointstotrack[2], 6, Scalar(231, 37, 18), -1, 8);
    circle(image_color, original1, 6, Scalar(231, 37, 18), -1, 8);
    circle(image_color, original2, 6, Scalar(231, 37, 18), -1, 8);
    circle(image_color, original2, 6, Scalar(231, 37, 18), -1, 8);

    line(image_color, pointstotrack[1], pointstotrack[2], CV_RGB(231, 37, 18), 4, CV_AA);
    line(image_color, original1, original2, CV_RGB(255, 0, 255), 4, CV_AA);
    line(image_color, pointstotrack[1], ref, CV_RGB(124,252,0), 4, CV_AA);




    outCvMat=image_color;






    IplImage outIplImage = outCvMat;
    //cvCvtColor(&outIplImage,&outIplImage, CV_BGR2RGB);     //CV_BGR2RGB

    ImageOf<PixelRgb> outYarpImg;
    outYarpImg.wrapIplImage(&outIplImage);

    pOutImg->prepare() = outYarpImg;
    pOutImg->write();




    char c;
    c=waitKey (10);
    if(c==32){
        aux = 0;
        lines.clear();
        pointstotrack.clear();
        original1 = Point(0, 0);
        original2 = Point(0, 0);
        times_lines=1;
    }




    image_color.release();
    swap(prev_image_grey, image_grey);
    swap(pointstotrack, prev_points);




    /*
    frame.release();
    lines_image.release();
    filtered_img.release();
    image.release();
    prev_image_grey.release();
    image_grey.release();
    image_color.release();

    cvReleaseImage( &inIplImage );  // release the memory for the image
    outCvMat.release();



}*/
//printf("[SegmentorThread] run()\n");






