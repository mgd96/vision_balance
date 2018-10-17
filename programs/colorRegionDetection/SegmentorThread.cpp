// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SegmentorThread.hpp"

namespace roboticslab
{

/************************************************************************/
void SegmentorThread::setIKinectDeviceDriver(yarp::dev::IOpenNI2DeviceDriver *_kinect) {
    kinect = _kinect;
}

/************************************************************************/
void SegmentorThread::setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg) {
    pOutImg = _pOutImg;
}

/************************************************************************/
void SegmentorThread::setOutPort(yarp::os::Port * _pOutPort) {
    pOutPort = _pOutPort;
}

/************************************************************************/
void SegmentorThread::init(yarp::os::ResourceFinder &rf) {

    fx_d = DEFAULT_FX_D;
    fy_d = DEFAULT_FY_D;
    cx_d = DEFAULT_CX_D;
    cy_d = DEFAULT_CY_D;
    fx_rgb = DEFAULT_FX_RGB;
    fy_rgb = DEFAULT_FY_RGB;
    cx_rgb = DEFAULT_CX_RGB;
    cy_rgb = DEFAULT_CY_RGB;

    algorithm = DEFAULT_ALGORITHM;
    locate = DEFAULT_LOCATE;
    maxNumBlobs = DEFAULT_MAX_NUM_BLOBS;
    morphClosing = DEFAULT_MORPH_CLOSING;
    morphOpening = DEFAULT_MORPH_OPENING;
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

        printf("\t--fx_d (default: \"%f\")\n",fx_d);
        printf("\t--fy_d (default: \"%f\")\n",fy_d);
        printf("\t--cx_d (default: \"%f\")\n",cx_d);
        printf("\t--cy_d (default: \"%f\")\n",cy_d);
        printf("\t--fx_rgb (default: \"%f\")\n",fx_rgb);
        printf("\t--fy_rgb (default: \"%f\")\n",fy_rgb);
        printf("\t--cx_rgb (default: \"%f\")\n",cx_rgb);
        printf("\t--cy_rgb (default: \"%f\")\n",cy_rgb);

        printf("\t--algorithm (default: \"%s\")\n",algorithm.c_str());
        printf("\t--locate (centroid or bottom; default: \"%s\")\n",locate.c_str());
        printf("\t--maxNumBlobs (default: \"%d\")\n",maxNumBlobs);
        printf("\t--morphClosing (percentage, 2 or 4 okay; default: \"%f\")\n",morphClosing);
        printf("\t--morphOpening (percentage, 2 or 4 okay; default: \"%f\")\n",morphOpening);
        printf("\t--outFeatures (mmX,mmY,mmZ,pxXpos,pxYpos,pxX,pxY,angle,area,aspectRatio,rectangularity,axisFirst,axisSecond \
               solidity,hue,sat,val,hueStdDev,satStdDev,valStdDev,time; \
                default: \"(%s)\")\n",outFeatures.toString().c_str());
        printf("\t--outFeaturesFormat (0=bottled,1=minimal; default: \"%d\")\n",outFeaturesFormat);
        printf("\t--outImage (0=rgb,1=bin; default: \"%d\")\n",outImage);
        printf("\t--rateMs (default: \"%d\")\n",rateMs);
        printf("\t--seeBounding (0=none,1=box,2=contour,3=both; default: \"%d\")\n",seeBounding);
        printf("\t--threshold (default: \"%d\")\n",threshold);
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    if (rf.check("fx_d")) fx_d = rf.find("fx_d").asDouble();
    if (rf.check("fy_d")) fy_d = rf.find("fy_d").asDouble();
    if (rf.check("cx_d")) cx_d = rf.find("cx_d").asDouble();
    if (rf.check("cy_d")) cy_d = rf.find("cy_d").asDouble();
    if (rf.check("fx_rgb")) fx_rgb = rf.find("fx_rgb").asDouble();
    if (rf.check("fy_rgb")) fy_rgb = rf.find("fy_rgb").asDouble();
    if (rf.check("cx_rgb")) cx_rgb = rf.find("cx_rgb").asDouble();
    if (rf.check("cy_rgb")) cy_rgb = rf.find("cy_rgb").asDouble();
    if (rf.check("algorithm")) algorithm = rf.find("algorithm").asString();
    if (rf.check("locate")) locate = rf.find("locate").asString();
    if (rf.check("maxNumBlobs")) maxNumBlobs = rf.find("maxNumBlobs").asInt();
    if (rf.check("morphClosing")) morphClosing = rf.find("morphClosing").asDouble();
    if (rf.check("morphOpening")) morphOpening = rf.find("morphOpening").asDouble();
    if (rf.check("outFeaturesFormat")) outFeaturesFormat = rf.find("outFeaturesFormat").asInt();

    printf("SegmentorThread using fx_d: %f, fy_d: %f, cx_d: %f, cy_d: %f.\n",
           fx_d,fy_d,cx_d,cy_d);
    printf("SegmentorThread using fx_rgb: %f, fy_rgb: %f, cx_rgb: %f, cy_rgb: %f.\n",
           fx_rgb,fy_rgb,cx_rgb,cy_rgb);
    printf("SegmentorThread using algorithm: %s, locate: %s.\n",
           algorithm.c_str(),locate.c_str());
    printf("SegmentorThread using maxNumBlobs: %d, morphClosing: %.2f, outFeaturesFormat: %d.\n",
           maxNumBlobs,morphClosing,outFeaturesFormat);

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

    if(cropSelector != 0) {
        processor.reset();
        inCropSelectorPort->setReader(processor);
    }

    this->setRate(rateMs);
    this->start();

}

/************************************************************************/
void SegmentorThread::run() {

    fx_d = DEFAULT_FX_D;
    fy_d = DEFAULT_FY_D;
    cx_d = DEFAULT_CX_D;
    cy_d = DEFAULT_CY_D;
    fx_rgb = DEFAULT_FX_RGB;
    fy_rgb = DEFAULT_FY_RGB;
    cx_rgb = DEFAULT_CX_RGB;
    cy_rgb = DEFAULT_CY_RGB;


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
    Mat webcamImage,gray,one;
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

    yarp::os::Bottle output_angles;
    travis:

    inCvMat.copyTo(webcamImage);


        //make a gray copy of the webcam image
        cvtColor(webcamImage, gray, COLOR_BGR2GRAY);




    if(option==0){
    cout<<"option 1:SolvePnP"<<endl;
    cout<<"option 2:PointCloud"<<endl;
    cin>>option;}



if(option==1){
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




         //detect chessboard corners
         bool found = findChessboardCorners(gray, cbSize, imagePoints, CALIB_CB_FAST_CHECK);


         //find camera orientation if the chessboard corners have been found
         if (found)
         {

             //find the camera extrinsic parameters
             Mat rotMatrix;
             //find the camera extrinsic parameters
             solvePnP(Mat(boardPoints), Mat(imagePoints), intrinsics, distortion, rvec, tvec, false);

             Rodrigues(rvec, rotMatrix);
             Vec3d  eulerAngles;
             Mat cameraMatrix, rotMatrix1, transVect, rotMatrixX, rotMatrixY, rotMatrixZ;
             double* _r = rotMatrix.ptr<double>();
             double projMatrix[12] = { _r[0], _r[1], _r[2], 0,
                 _r[3], _r[4], _r[5], 0,
                 _r[6], _r[7], _r[8], 0 };

             decomposeProjectionMatrix(Mat(3, 4, CV_64FC1, projMatrix),
                 cameraMatrix,
                 rotMatrix1,
                 transVect,
                 rotMatrixX,
                 rotMatrixY,
                 rotMatrixZ,
                 eulerAngles);

             double yaw = eulerAngles[1];
             double pitch = eulerAngles[0];
             double roll = eulerAngles[2];
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



             std::ostringstream strs;
             if (pitch<0){
                 strs << "rot X: " << (pitch+180);
                 output_angles.addDouble(pitch+180); }
             else if (pitch>0){
                 strs << "rot X: " << (pitch - 180);
                 output_angles.addDouble(pitch-180);}
             else  {
                 strs << "rot X: " << pitch;
                  output_angles.addDouble(pitch);}
             std::string str = strs.str();

             std::ostringstream strs2;
             strs2 << "rot Y: " << yaw;
             output_angles.addDouble(yaw);
             std::string str2 = strs2.str();

             std::ostringstream strs3;
             strs3 << "rot Z: " << roll;
             output_angles.addDouble(roll);
             std::string str3 = strs3.str();

             /*         } else if ( outFeatures.get(elem).asString() == "mmZ" ) {
                          if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                              output.addDouble(mmZ[0]);
                          } else {
                              yarp::os::Bottle locZs;
                              for (int i = 0; i < blobsXY.size(); i++)
                                  locZs.addDouble(mmZ[i]);
                              output.addList() = locZs;
                          }*/

             putText(webcamImage, str, Point(60, 60), FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0), 2, 8, false);
             putText(webcamImage, str2, Point(60, 90), FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 255, 0), 2, 8, false);
             putText(webcamImage, str3, Point(60, 120), FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 0, 255), 2, 8, false);
             cout<<"x: "<<str<<" y: "<<str2<<" z: "<<str3<<endl;

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








     }
   }
if(option==2){

    //aqui codigo pointcloud

}


outCvMat=webcamImage;



IplImage outIplImage = outCvMat;
//cvCvtColor(&outIplImage,&outIplImage, CV_BGR2RGB);     //CV_BGR2RGB

yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg;

outYarpImg.wrapIplImage(&outIplImage);

pOutImg->prepare() = outYarpImg;
pOutImg->write();

//show the gray image
//namedWindow("Gray Image", CV_WINDOW_AUTOSIZE);
//imshow("Gray Image", gray);

cvReleaseImage( &inIplImage );  // release the memory for the image

if (output_angles.size() > 0)
    pOutPort->write(output_angles);

outCvMat.release();

 }
}







