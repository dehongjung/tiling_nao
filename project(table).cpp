// DYROS - Dynamic Robotic System Lab
//ROBOT SIT

// Aldebaran includes.
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alproxies/altexttospeechproxy.h>
#include <alvalue/alvalue.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>

// Math includes
#include <almath/types/alpose2d.h>
#include <almath/tools/aldubinscurve.h>
#include <almath/tools/almathio.h>
#include <almath/tools/altrigonometry.h>

// Opencv includes.
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <stdio.h>
#include <pthread.h> 
#include <cstdlib>


#include <qi/os.hpp>

using namespace AL;
using namespace cv;
using namespace std;

char mouseController[] = "Raw Video";
char objWindow[] = "Object Window";
char scribbleWindow[] = "Scribble Window";
char resultWindow[] = "Mouse Controller";
char naoIP[] = "192.168.0.10";

Mat img_rgb,img_gray,canny_output,drawing;
int thresh = 100;
int max_thresh = 255;
int DARKBLUEflag = 0, BLACKflag = 0, YELLOWflag = 0, WHITEflag = 0, ORANGEflag = 0, WALKflag = 0, PXflag = 0;
float horizontal_pixels = 0;
float x=0, y=0, theta=0;
int BLACKdone = 0, ORANGEdone = 0, DARKBLUEdone = 0;
int stop = 0; 
float mais = 0.0f;
int rh = 255, rl = 0, gh = 255, gl = 0, bh = 255, bl = 0;


void *Walk(void *threadid)
{
  AL::ALMotionProxy motion(naoIP, 9559);
  AL::ALRobotPostureProxy postureProxy(naoIP, 9559);
  
  long stop, stop2;
  stop = (long)threadid;
  stop2 = stop;

  if(stop == 0){      
       motion.angleInterpolation("HeadYaw", 1.57f - mais, 0.7f, true);
      //proxy->waitUntilMoveIsFinished();
       if (mais > 2.90f){
	 stop = 3;
	 //motion.stiffnessInterpolation("HeadYaw", 0.0f, 1.0f);
       }
	 
  }

   
   
   //pthread_exit(NULL);
}

/*void Calibration(void *imgHeader){
  string windowName = "background";
  namedWindow(windowName);
  
  createTrackbar("rh", windowName, &rh, 255);
  createTrackbar("rl", windowName, &rl, 255);
  createTrackbar("gh", windowName, &gh, 255);
  createTrackbar("gl", windowName, &gl, 255);
  createTrackbar("bh", windowName, &bh, 255);
  createTrackbar("bl", windowName, &bl, 255);
  
   Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
   Mat bgIsolation;
   int key = 0;
   
   inRange(imgHeader, Scalar(bl, gl, rl), Scalar(bh, gh, rh), bgIsolation);
   
   imshow(windowName, bgIsolation);
  
}*/


int main(int argc, char* argv[]) {
  static int DARKBLUEposX, DARKBLUEposY, DARKBLUElastX, DARKBLUElastY; //To hold the X and Y position of tracking color object
  static int ORANGEposX, ORANGEposY, ORANGElastX, ORANGElastY, distance;
  static int WHITEposX, WHITEposY, WHITElastX, WHITElastY;
  static int BLACKposX, BLACKposY, BLACKlastX, BLACKlastY;
  static int YELLOWposX, YELLOWposY, YELLOWlastX, YELLOWlastY;
  int i, j;
  int step = 1;
  
  float BLACKAngle;
  float ORANGEAngle;
  float DARKBLUEAngle;
    float BLACKAngle2;
  float ORANGEAngle2;
  float DARKBLUEAngle2;
  float ORANGEAngle3;
  //struct thread_data td;
  
  //vector<Point2f> vert(6);

  AL::ALTextToSpeechProxy fTtsProxy(argv[1], 9559);
  AL::ALMotionProxy motion(argv[1], 9559);
  AL::ALRobotPostureProxy postureProxy(argv[1], 9559);
  if (argc < 2)
  {
    std::cerr << "Usage 'getimages robotIp'" << std::endl;
    return 1;
  }
  
  
  pthread_t threads;

  
  
  //STIFFNESS ON
  AL::ALValue jointNames = AL::ALValue::array("LShoulderPitch", "LElbowYaw", "LElbowRoll");
  AL::ALValue jointNames2 = AL::ALValue::array("LShoulderPitch","LShoulderRoll", "LElbowYaw", "LElbowRoll");
  AL::ALValue jointNames3 = AL::ALValue::array("LShoulderRoll", "RShoulderRoll");
  AL::ALValue RjointNames = AL::ALValue::array("RShoulderPitch", "RElbowYaw", "RElbowRoll");
  AL::ALValue RjointNames2 = AL::ALValue::array("RShoulderPitch","RShoulderRoll", "RElbowYaw", "RElbowRoll");
  AL::ALValue BjointNames = AL::ALValue::array("LShoulderPitch", "LElbowYaw", "LElbowRoll", "RShoulderPitch", "RElbowYaw", "RElbowRoll");
  AL::ALValue BjointNames2 = AL::ALValue::array("LShoulderPitch","LShoulderRoll", "LElbowYaw", "LElbowRoll", "RShoulderPitch","RShoulderRoll", "RElbowYaw", "RElbowRoll");
  
  float stiffnessLists = 1.0f;
  float timeLists      = 1.0f;
  motion.stiffnessInterpolation("Body", stiffnessLists, timeLists);
  //motion.stiffnessInterpolation("HeadYaw", 0.0f, timeLists);
  //motion.moveInit();
  //motion.angleInterpolation("HeadPitch", 0.0f, 1.0f, true);
  
  
  //motion.angleInterpolationWithSpeed("HeadPitch", 0.5f, 0.6f);
  
  
  const std::string robotIp(argv[1]);
  
  CvMemStorage* storage = cvCreateMemStorage(0);
  /** Create a proxy to ALVideoDevice on the robot.*/
  ALVideoDeviceProxy camProxy(robotIp, 9559);

  /** Subscribe a client image requiring 320*240 and BGR colorspace.*/
  const std::string clientName = camProxy.subscribeCamera("testsgss", kBottomCamera, kQVGA, kBGRColorSpace, 30);
  
  /**Change Camera */
  camProxy.setParam(kCameraSelectID,1);  //CHANGE TO THE BOTTOM CAMERA (0 - TOP, 1 - BOTTOM)
  

  
  /** Create an cv::Mat header to wrap into an opencv image.*/
  cv::Mat imgHeader = cv::Mat(cv::Size(320, 240), CV_8UC3);
  //cv::Mat imgHeader = cv::Mat(cv::Size(640, 480), CV_8UC3);
  
  /** Create a OpenCV window to display the images. */
  //cv::namedWindow("images");
  
  
  //Mat DARKBLUETrackingFrames, GREENTrackingFrames, BLUETrackingFrames; //Matrixes to hold color filtered Frames in GRAY color space
  Mat WHITETrackingFrames, DARKBLUETrackingFrames, ORANGETrackingFrames, YELLOWTrackingFrames, BLACKTrackingFrames;
  
  Mat resutantFrame; //Matrix to add RAW and user scribble Frames
  
  //CvMoments DARKBLUEMoment; //Structure to hold moments information and their order
  //CvMoments GREENMoment;
  //CvMoments BLUEMoment;
  
  CvMoments DARKBLUEMoment;
  CvMoments WHITEMoment;
  CvMoments ORANGEMoment;
  CvMoments BLACKMoment;
  CvMoments YELLOWMoment; 
  
  Mat *scribbleFrame = new Mat(imgHeader.rows, imgHeader.cols, CV_8UC3); //A matrix to hold the user scribble on the screen
  
  motion.angleInterpolation("HeadYaw", 1.57f, 1.0f, true);
  qi::os::msleep(1000);
  
  while ((char) cv::waitKey(30) != 27 && step < 4) {
    ALValue img = camProxy.getImageRemote(clientName);
    imgHeader.data = (uchar*) img[6].GetBinary();
    
    motion.angleInterpolation("HeadYaw", 1.57f - mais, 1.0f, true);
    
    cvtColor(imgHeader,img_gray,CV_RGB2GRAY);
    
    //Calibration(*imgHeader);
    
    //inRange(imgHeader, Scalar(bl, gl, rl), Scalar(bh, gh, rh), bgIsolation);
    //cvtColor(imgHeader,img_hsv,CV_RGB2HSV);
    
    //imgHeader >> imgHeader; //Each frames is copied to our rgbCameraFrames object
    
    //GaussianBlur(imgHeader, DARKBLUETrackingFrames, Size(11, 11), 0, 0); //Just a filter to reduce the noise
    //GaussianBlur(imgHeader, GREENTrackingFrames, Size(11, 11), 0, 0);
    //GaussianBlur(imgHeader, BLUETrackingFrames, Size(11, 11), 0, 0);
    
    GaussianBlur(imgHeader, DARKBLUETrackingFrames, Size(11, 11), 0, 0); //Just a filter to reduce the noise
    GaussianBlur(imgHeader, WHITETrackingFrames, Size(11, 11), 0, 0);
    GaussianBlur(imgHeader, ORANGETrackingFrames, Size(11, 11), 0, 0);
    GaussianBlur(imgHeader, YELLOWTrackingFrames, Size(11, 11), 0, 0);
    GaussianBlur(imgHeader, BLACKTrackingFrames, Size(11, 11), 0, 0);
    
    //           B  G  R     B   G    R
    //inRange(GREENTrackingFrames, Scalar(0,192,0), Scalar(255, 255, 157),GREENTrackingFrames); //GREEN
    //inRange(BLUETrackingFrames, Scalar(150,0,0), Scalar(255, 255, 119),BLUETrackingFrames); //BLUE
    //inRange(DARKBLUETrackingFrames, Scalar(0,0,172), Scalar(145, 255, 255),DARKBLUETrackingFrames); //DARKBLUE
    
    //inRange(ORANGETrackingFrames, Scalar(0,0,194), Scalar(151, 166, 255),ORANGETrackingFrames); //ORANGE
    inRange(BLACKTrackingFrames, Scalar(23,4,9), Scalar(55, 30, 42),BLACKTrackingFrames);  //BLACK
    //inRange(DARKBLUETrackingFrames, Scalar(109,0,0), Scalar(255, 68, 118),DARKBLUETrackingFrames);  //DARK BLUE
    //inRange(YELLOWTrackingFrames, Scalar(22,230,0), Scalar(190, 255, 255),YELLOWTrackingFrames);  //YELLOW
    inRange(WHITETrackingFrames, Scalar(223,231,169), Scalar(255, 255, 255),WHITETrackingFrames);  //WHITE
    
    inRange(DARKBLUETrackingFrames, Scalar(111,28,37), Scalar(171, 71, 95),DARKBLUETrackingFrames);  //DARK BLUE
    inRange(YELLOWTrackingFrames, Scalar(119,177,108), Scalar(165, 221, 206),YELLOWTrackingFrames);  //YELLOW
    inRange(ORANGETrackingFrames, Scalar(81,119,170), Scalar(128, 156, 236),ORANGETrackingFrames); //ORANGE
    
    //cv::subtract(cv::Scalar::all(255),colorTrackingFrames,colorTrackingFrames);
    
    //DARKBLUEMoment = moments(DARKBLUETrackingFrames); //We give the binary converted frames for calculating the moments
    //GREENMoment = moments(GREENTrackingFrames);
    //BLUEMoment = moments(BLUETrackingFrames);
    
    DARKBLUEMoment = moments(DARKBLUETrackingFrames);
    YELLOWMoment = moments(YELLOWTrackingFrames);
    ORANGEMoment = moments(ORANGETrackingFrames);
    WHITEMoment = moments(WHITETrackingFrames);
    BLACKMoment = moments(BLACKTrackingFrames);
    
    
    double DARKBLUEmoment10 = cvGetSpatialMoment(&DARKBLUEMoment, 1, 0); //Sum of X coordinates of all white pixels
    double DARKBLUEmoment01 = cvGetSpatialMoment(&DARKBLUEMoment, 0, 1); //Sum of Y coordinates of all white pixels
    double DARKBLUEarea = cvGetCentralMoment(&DARKBLUEMoment, 0, 0);
    double WHITEmoment10 = cvGetSpatialMoment(&WHITEMoment, 1, 0); //Sum of X coordinates of all white pixels
    double WHITEmoment01 = cvGetSpatialMoment(&WHITEMoment, 0, 1); //Sum of Y coordinates of all white pixels
    double WHITEarea = cvGetCentralMoment(&WHITEMoment, 0, 0);
    double ORANGEmoment10 = cvGetSpatialMoment(&ORANGEMoment, 1, 0); //Sum of X coordinates of all white pixels
    double ORANGEmoment01 = cvGetSpatialMoment(&ORANGEMoment, 0, 1); //Sum of Y coordinates of all white pixels
    double ORANGEarea = cvGetCentralMoment(&ORANGEMoment, 0, 0);
    double YELLOWmoment10 = cvGetSpatialMoment(&YELLOWMoment, 1, 0); //Sum of X coordinates of all white pixels
    double YELLOWmoment01 = cvGetSpatialMoment(&YELLOWMoment, 0, 1); //Sum of Y coordinates of all white pixels
    double YELLOWarea = cvGetCentralMoment(&YELLOWMoment, 0, 0);
    double BLACKmoment10 = cvGetSpatialMoment(&BLACKMoment, 1, 0); //Sum of X coordinates of all white pixels
    double BLACKmoment01 = cvGetSpatialMoment(&BLACKMoment, 0, 1); //Sum of Y coordinates of all white pixels
    double BLACKarea = cvGetCentralMoment(&BLACKMoment, 0, 0);
    //printf("1. x-Axis moments %f  y-Axis moments %f  Area of the moment  %f\n",
    //						   moment10, moment01, area);
    //printf("2. x movement %f  y movement %f \n\n", moment10 / area,
    //    moment01 / area);
    //  From terminal :
    //  1. x-Axis moments 760645620.000000  y-Axis moments 631169625.000000  Area of the moment  1994355.000000
    //  2. x movement 381.399310  y movement 316.478072
    
    DARKBLUElastX = DARKBLUEposX;
    DARKBLUElastY = DARKBLUEposY;
    DARKBLUEposX = (DARKBLUEmoment10 / DARKBLUEarea);
    DARKBLUEposY = DARKBLUEmoment01 / DARKBLUEarea;
    
    
    
    YELLOWlastX = YELLOWposX;
    YELLOWlastY = YELLOWposY;
    YELLOWposX = (YELLOWmoment10 / YELLOWarea);
    YELLOWposY = YELLOWmoment01 / YELLOWarea;
    
    ORANGElastX = ORANGEposX;
    ORANGElastY = ORANGEposY;
    ORANGEposX = (ORANGEmoment10 / ORANGEarea);
    ORANGEposY = ORANGEmoment01 / ORANGEarea;
    
    BLACKlastX = BLACKposX;
    BLACKlastY = BLACKposY;
    BLACKposX = (BLACKmoment10 / BLACKarea);
    BLACKposY = BLACKmoment01 / BLACKarea;
    
    WHITElastX = WHITEposX;
    WHITElastY = WHITEposY;
    WHITEposX = (WHITEmoment10 / WHITEarea);
    WHITEposY = WHITEmoment01 / WHITEarea;
  
    horizontal_pixels = WHITElastX - 120;
    distance = -0.332*WHITElastY + 109.82;     //Collected some experimental data (measured the distance bewtween the robot and the square according to the pixel position on the camera image), plotted it and did a Linear Regreesssion to get the equation
    
    //pthread_create(&threads, NULL, Walk, (void *)stop);
    

    
    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    Canny( img_gray, canny_output, thresh, thresh*2, 3 );
    imshow("Canny",canny_output);
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    
    vector<Point> approxTriangle;
    
    //int DARKBLUEcheck = pointPolygonTest( contours[0], Point2f(i,j), true );
    
    if (DARKBLUEposX > 0 && DARKBLUEposY > 0 && DARKBLUElastX > 0 && DARKBLUElastY >> 0 && DARKBLUEdone == 0) {
      
      line(*scribbleFrame, cvPoint(DARKBLUEposX, DARKBLUEposY), cvPoint(DARKBLUElastX, DARKBLUElastY),
	   cvScalar(0, 255, 255), 1); //To draw a continuous stretch of lines
      line(imgHeader, cvPoint(DARKBLUEposX, DARKBLUEposY), cvPoint(DARKBLUElastX, DARKBLUElastY),
	   cvScalar(0, 255, 255), 5); //To draw a yello point on the center of the colored object
      //cvPoint is used to create a Point data type which holds the pixel location
      //printf("Contour size :%d \n", contours.size());
      
      for(size_t i = 0; i < contours.size(); i++){
	approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);
	if(approxTriangle.size() == 4){
	  if(contourArea(contours[i]) > 700){
	    
	    int DARKBLUEcheck = pointPolygonTest( contours[i], Point2f(DARKBLUElastX, DARKBLUElastY), true );
	    //printf("BLUE CHECK: %d \n", DARKBLUEcheck);
	    
	    if(DARKBLUEcheck >= 0){
	      //printf("DARKBLUE Contour area: %f",contourArea(contours[i]) );
	      //printf("\n");
	      drawContours(drawing, contours, i, Scalar(0, 255, 255), CV_FILLED); // fill GREEN
	      vector<Point>::iterator vertex;
	      for(vertex = approxTriangle.begin(); vertex != approxTriangle.end(); ++vertex){
		circle(drawing, *vertex, 3, Scalar(0, 0, 255), 1);
	      } 
	      fTtsProxy.say("One dark blue square detected");
	      DARKBLUEflag++;
	      WHITEflag = 0;
	      ORANGEflag = 0;
	      YELLOWflag = 0;
	      BLACKflag = 0;
	      //WALKflag = 0;
	      PXflag = 1;
	      
	      step++;
	      //stop = 1;
	      
	      DARKBLUEAngle = 1.57f - mais;
	      
	      if(DARKBLUEflag > 0){
		DARKBLUEdone = 1;
		stop = 0;
	      }
	      break;
	    }
	  }
	}
      }
      /*if (DARKBLUEflag>2){
	
	    WALKflag++;
	    DARKBLUEflag = 0;
	    
      }*/
      
    }
     if (ORANGEposX > 0 && ORANGEposY > 0 && ORANGElastX > 0 && ORANGElastY >> 0 && DARKBLUEdone == 1 && ORANGEdone == 0) {
      
      line(*scribbleFrame, cvPoint(ORANGEposX, ORANGEposY), cvPoint(ORANGElastX, ORANGElastY),
	   cvScalar(0, 255, 255), 1); //To draw a continuous stretch of lines
      line(imgHeader, cvPoint(ORANGEposX, ORANGEposY), cvPoint(ORANGElastX, ORANGElastY),
	   cvScalar(0, 255, 255), 5); //To draw a yello point on the center of the colored object
      //cvPoint is used to create a Point data type which holds the pixel location
      
      for(size_t i = 0; i < contours.size(); i++){
	approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);
	if(approxTriangle.size() == 4){
	  if(contourArea(contours[i]) > 700){
	    
	    int ORANGEcheck = pointPolygonTest( contours[i], Point2f(ORANGElastX, ORANGElastY), true );
	    
	    if(ORANGEcheck >= 0){
	    
	      //printf("ORANGE Contour area: %f",contourArea(contours[i]) );
	      //printf("\n");
	      drawContours(drawing, contours, i, Scalar(0, 255, 255), CV_FILLED); // fill GREEN
	      vector<Point>::iterator vertex;
	      for(vertex = approxTriangle.begin(); vertex != approxTriangle.end(); ++vertex){
		circle(drawing, *vertex, 3, Scalar(0, 0, 255), 1);
	      }  
	      fTtsProxy.say("One orange square detected");
	      ORANGEflag++;
	      DARKBLUEflag = 0;
	      WHITEflag = 0;
	      YELLOWflag = 0;
	      BLACKflag = 0;
	      //WALKflag = 0;
	      stop = 1;
	      step++;
	      
	      ORANGEAngle = 1.57f - mais;
	      
	      if(ORANGEflag > 0){
		ORANGEdone = 1;
		stop = 0;
	      }
	      
	      break;
	    }
	  }
	}
      }
      /*if(GREENflag > 2){

	    
	    GREENflag = 0;
      }*/
      
    }
    /*else if (WHITEposX > 0 && WHITEposY > 0 && WHITElastX > 0 && WHITElastY >> 0) {
      
      
      line(*scribbleFrame, cvPoint(WHITEposX, WHITEposY), cvPoint(WHITElastX, WHITElastY),
	   cvScalar(0, 255, 255), 1); //To draw a continuous stretch of lines
      line(imgHeader, cvPoint(WHITEposX, WHITEposY), cvPoint(WHITElastX, WHITElastY),
	   cvScalar(0, 255, 255), 5); //To draw a yello point on the center of the colored object
      //cvPoint is used to create a Point data type which holds the pixel location
      
      for(size_t i = 0; i < contours.size(); i++){
	approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);
	if(approxTriangle.size() == 4){
	  if(contourArea(contours[i]) > 700){
	    
	    int WHITEcheck = pointPolygonTest( contours[i], Point2f(WHITElastX, WHITElastY), true );
	    
	    if(WHITEcheck >= 0){
	      
	      printf("WHITE Contour area: %f",contourArea(contours[i]) );
	      printf("\n");
	      drawContours(drawing, contours, i, Scalar(0, 255, 255), CV_FILLED); // fill GREEN
	      vector<Point>::iterator vertex;
	      for(vertex = approxTriangle.begin(); vertex != approxTriangle.end(); ++vertex){
		circle(drawing, *vertex, 3, Scalar(0, 0, 255), 1);
	      }  
	      fTtsProxy.say("One white square detected");
	      ORANGEflag = 0;
	      DARKBLUEflag = 0;
	      YELLOWflag = 0;
	      BLACKflag = 0;
	      WHITEflag++;
	      //WALKflag = 0;
	      break;
	    }
	  }
	}
      }
      if (WHITEflag>2){
	
	    WALKflag++;
	    WHITEflag = 0;
	    
      }
    }*/
     if (BLACKposX > 0 && BLACKposY > 0 && BLACKlastX > 0 && BLACKlastY >> 0 && DARKBLUEdone == 1 && ORANGEdone ==1 && BLACKdone == 0) {
      
      
      line(*scribbleFrame, cvPoint(BLACKposX, BLACKposY), cvPoint(BLACKlastX, BLACKlastY),
	   cvScalar(0, 255, 255), 1); //To draw a continuous stretch of lines
      line(imgHeader, cvPoint(BLACKposX, BLACKposY), cvPoint(BLACKlastX, BLACKlastY),
	   cvScalar(0, 255, 255), 5); //To draw a yello point on the center of the colored object
      //cvPoint is used to create a Point data type which holds the pixel location
      
      for(size_t i = 0; i < contours.size(); i++){
	approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);
	if(approxTriangle.size() == 4){
	  if(contourArea(contours[i]) > 700){
	    
	    int BLACKcheck = pointPolygonTest( contours[i], Point2f(BLACKlastX, BLACKlastY), true );
	    
	    if(BLACKcheck >= 0){
	      
	      //printf("BLACK Contour area: %f",contourArea(contours[i]) );
	      //printf("\n");
	      drawContours(drawing, contours, i, Scalar(0, 255, 255), CV_FILLED); // fill GREEN
	      vector<Point>::iterator vertex;
	      for(vertex = approxTriangle.begin(); vertex != approxTriangle.end(); ++vertex){
		circle(drawing, *vertex, 3, Scalar(0, 0, 255), 1);
	      }  
	      fTtsProxy.say("One black square detected");
	      ORANGEflag = 0;
	      DARKBLUEflag = 0;
	      WHITEflag = 0;
	      YELLOWflag = 0;
	      BLACKflag++;
	      //WALKflag = 0;
	      stop = 1;
	      step++;
	      
	      BLACKAngle = 1.57f - mais;
	      
	      if(BLACKflag > 0){
		BLACKdone = 1;
		stop = 0;
	      }
	      break;
	    }
	  }
	}
      }
    }
    /*else if (YELLOWposX > 0 && YELLOWposY > 0 && YELLOWlastX > 0 && YELLOWlastY >> 0) {
      
      
      line(*scribbleFrame, cvPoint(YELLOWposX, YELLOWposY), cvPoint(YELLOWlastX, YELLOWlastY),
	   cvScalar(0, 255, 255), 1); //To draw a continuous stretch of lines
      line(imgHeader, cvPoint(YELLOWposX, YELLOWposY), cvPoint(YELLOWlastX, YELLOWlastY),
	   cvScalar(0, 255, 255), 5); //To draw a yello point on the center of the colored object
      //cvPoint is used to create a Point data type which holds the pixel location
      
      for(size_t i = 0; i < contours.size(); i++){
	approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);
	if(approxTriangle.size() == 4){
	  if(contourArea(contours[i]) > 1000){
	    drawContours(drawing, contours, i, Scalar(0, 255, 255), CV_FILLED); // fill GREEN
	    vector<Point>::iterator vertex;
	    for(vertex = approxTriangle.begin(); vertex != approxTriangle.end(); ++vertex){
	      circle(drawing, *vertex, 3, Scalar(0, 0, 255), 1);
	    }  
	    fTtsProxy.say("One yellow square detected");
	    ORANGEflag = 0;
	    DARKBLUEflag = 0;
	    WHITEflag = 0;
	    BLACKflag = 0;
	    YELLOWflag++;
	    //WALKflag = 0;
	    break;
	  }
	}
      }
    }*/
    
      /*if(BLUEflag > 2){

	    
	    BLUEflag = 0;
      }
      
    }else{
      DARKBLUEflag = 0;
      GREENflag = 0;
      BLUEflag = 0;
    }*/
    
      
    mais = mais + 0.10f;
    if (mais > 2.67f)
      break;

    imshow("Squares",drawing);

    
    imshow(objWindow, DARKBLUETrackingFrames);
    imshow(objWindow, WHITETrackingFrames);
    imshow(objWindow, ORANGETrackingFrames);
    imshow(objWindow, BLACKTrackingFrames);
    imshow(objWindow, YELLOWTrackingFrames);
    
    camProxy.releaseImage(clientName);
    
    //imshow(scribbleWindow, *scribbleFrame);
    imshow(mouseController, imgHeader);
    //imshow(windowName, bgIsolation);
    //imshow(objWindow, colorTrackingFrames);
    add(imgHeader, *scribbleFrame, resutantFrame); //Add two Matrix of the same size
    imshow(resultWindow, resutantFrame);
    //waitKey(1); // OpenCV way of adding a delay, generally used to get a Key info.
    //if (WALKflag == 4)
      //break;
  }
  std::cout << "DARKBLUE angle: " << DARKBLUEAngle << std::endl;
  std::cout << "ORANGE angle: " << ORANGEAngle << std::endl;
  std::cout << "BLACK angle: " << BLACKAngle << std::endl;
  
  float TargetAngle;
  
  while(true){
 
    //LEFT ARM----------------------------------------------------
    AL::ALValue arm1 = AL::ALValue::array(
      -0.1f ,  // "LShoulderPitch"
      +0.61f ,  // "LShoulderRoll"
      +1.58f ,  // "LElbowYaw"
      -0.02f ); // "LElbowRoll"
    AL::ALValue arm2 = AL::ALValue::array(
      +0.61f);  // "LShoulderRoll"    
    AL::ALValue arm3 = AL::ALValue::array(
      +0.39f ,  // "LShoulderPitch"     
      +1.58f ,  // "LElbowYaw"
      -0.02f ); // "LElbowRoll"
    AL::ALValue arm4 = AL::ALValue::array(
      -0.1f ,  // "LShoulderPitch"      
      +1.58f ,  // "LElbowYaw"
      -0.02f ); // "LElbowRoll" 
    AL::ALValue arm5 = AL::ALValue::array(
      -0.1f ,  // "LShoulderPitch"
      +0.61f ,  // "LShoulderRoll"
      +1.58f ,  // "LElbowYaw"
      -0.02f ); // "LElbowRoll"
     
    //RIGHT ARM---------------------------------------------------  
    AL::ALValue Rarm1 = AL::ALValue::array(
      -0.1f ,  // "RShoulderPitch"
      -0.61f ,  // "RShoulderRoll"
      -1.58f ,  // "RElbowYaw"
      +0.02f ); // "RElbowRoll"
    AL::ALValue Rarm2 = AL::ALValue::array(
      -0.61f);  // "RShoulderRoll"    
    AL::ALValue Rarm3 = AL::ALValue::array(
      +0.39f ,  // "RShoulderPitch"     
      -1.58f ,  // "RElbowYaw"
      +0.02f ); // "RElbowRoll"
    AL::ALValue Rarm4 = AL::ALValue::array(
      -0.1f ,  // "RShoulderPitch"      
      -1.58f ,  // "RElbowYaw"
      +0.02f ); // "RElbowRoll" 
    AL::ALValue Rarm5 = AL::ALValue::array(
      -0.1f ,  // "RShoulderPitch"
      -0.61f ,  // "RShoulderRoll"
      -1.58f ,  // "RElbowYaw"
      +0.02f ); // "RElbowRoll"
      
      //BOTH ARMS
    AL::ALValue Barm1 = AL::ALValue::array(
      -0.1f ,  // "LShoulderPitch"
      +0.61f ,  // "LShoulderRoll"
      +1.58f ,  // "LElbowYaw"
      -0.02f, // "LElbowRoll"
      -0.1f ,  // "RShoulderPitch"
      -0.61f ,  // "RShoulderRoll"
      -1.58f ,  // "RElbowYaw"
      +0.02f ); // "RElbowRoll"
    AL::ALValue Barm2 = AL::ALValue::array(
      +0.61f,  // "LShoulderRoll"    
      -0.61f);  // "RShoulderRoll"  
    AL::ALValue Barm3 = AL::ALValue::array(
      +0.39f ,  // "LShoulderPitch"     
      +1.58f ,  // "LElbowYaw"
      -0.02f, // "LElbowRoll"
      +0.39f ,  // "RShoulderPitch"     
      -1.58f ,  // "RElbowYaw"
      +0.02f ); // "RElbowRoll"
    AL::ALValue Barm4 = AL::ALValue::array(
      -0.1f ,  // "LShoulderPitch"      
      +1.58f ,  // "LElbowYaw"
      -0.02f, // "LElbowRoll" 
      -0.1f ,  // "RShoulderPitch"      
      -1.58f ,  // "RElbowYaw"
      +0.02f ); // "RElbowRoll" 
    AL::ALValue Barm5 = AL::ALValue::array(
      -0.1f ,  // "LShoulderPitch"
      +0.61f ,  // "LShoulderRoll"
      +1.58f ,  // "LElbowYaw"
      -0.02f , // "LElbowRoll"
      -0.1f ,  // "RShoulderPitch"
      -0.61f ,  // "RShoulderRoll"
      -1.58f ,  // "RElbowYaw"
      +0.02f ); // "RElbowRoll"
      
    AL::ALValue targetTimes = AL::ALValue::array(0.6f, 0.6f,0.6f);
    AL::ALValue targetTimes2 = AL::ALValue::array(0.6f, 0.6f,0.6f, 0.6f);
    AL::ALValue BtargetTimes = AL::ALValue::array(0.6f, 0.6f,0.6f, 0.6f, 0.6f,0.6f);
    AL::ALValue BtargetTimes2 = AL::ALValue::array(0.6f, 0.6f,0.6f, 0.6f, 0.6f, 0.6f,0.6f, 0.6f);
    
    
    char key = getchar();
    
    if (key == 27){
      motion.stiffnessInterpolation("Body", 0.0f, timeLists);
      break;
    }
      
      
    switch(key){
      case 'a':       //DARKBLUE
	std::cout << "DARKBLUE selected" << std::endl;

	if (DARKBLUEAngle > 0.1){
	  DARKBLUEAngle2 = DARKBLUEAngle - 0.37;
	  arm2 = AL::ALValue::array(DARKBLUEAngle2);  // "LShoulderRoll"
		    
	  motion.angleInterpolation(jointNames2, arm1, targetTimes2 , true);
	  motion.angleInterpolation("LShoulderRoll", arm2, 0.6f, true);
	  motion.angleInterpolation(jointNames, arm3, targetTimes, true);
	  qi::os::msleep(1000);
	  motion.angleInterpolation(jointNames, arm4, targetTimes, true);
	  motion.angleInterpolation(jointNames2, arm5, targetTimes2 , true);
	}
	if (DARKBLUEAngle < 0.1 && DARKBLUEAngle > -0.06){
	  DARKBLUEAngle2 = DARKBLUEAngle - 0.37;
	  arm2 = AL::ALValue::array(DARKBLUEAngle2);  // "LShoulderRoll"
		    
	  motion.angleInterpolation(jointNames2, arm1, targetTimes2 , true);
	  motion.angleInterpolation("LShoulderRoll", arm2, 0.6f, true);
	  motion.angleInterpolation(jointNames, arm3, targetTimes, true);
	  qi::os::msleep(1000);
	  motion.angleInterpolation(jointNames, arm4, targetTimes, true);
	  motion.angleInterpolation(jointNames2, arm5, targetTimes2 , true);

	}
	if(DARKBLUEAngle < -0.06){
	  DARKBLUEAngle2 = DARKBLUEAngle + 0.37;
	  Rarm2 = AL::ALValue::array(DARKBLUEAngle2);  // "LShoulderRoll"
		    
	  motion.angleInterpolation(RjointNames2, Rarm1, targetTimes2 , true);
	  motion.angleInterpolation("RShoulderRoll", Rarm2, 0.6f, true);
	  motion.angleInterpolation(RjointNames, Rarm3, targetTimes, true);
	  qi::os::msleep(1000);
	  motion.angleInterpolation(RjointNames, Rarm4, targetTimes, true);
	  motion.angleInterpolation(RjointNames2, Rarm5, targetTimes2 , true);	  
	}
	break;
	
      case 'p':      //BLACK
	std::cout << "BLACK selected" << std::endl;
	
	if (BLACKAngle > 0.1){
	  BLACKAngle2 = BLACKAngle - 0.37;
	  arm2 = AL::ALValue::array(BLACKAngle2);  // "LShoulderRoll"
		    
	  motion.angleInterpolation(jointNames2, arm1, targetTimes2 , true);
	  motion.angleInterpolation("LShoulderRoll", arm2, 0.6f, true);
	  motion.angleInterpolation(jointNames, arm3, targetTimes, true);
	  qi::os::msleep(1000);
	  motion.angleInterpolation(jointNames, arm4, targetTimes, true);
	  motion.angleInterpolation(jointNames2, arm5, targetTimes2 , true);
	}
	if (BLACKAngle < 0.1 && BLACKAngle > -0.06){
	  BLACKAngle2 = BLACKAngle - 0.37;
	  arm2 = AL::ALValue::array(BLACKAngle2);  // "LShoulderRoll"
		    
	  motion.angleInterpolation(jointNames2, arm1, targetTimes2 , true);
	  motion.angleInterpolation("LShoulderRoll", arm2, 0.6f, true);
	  motion.angleInterpolation(jointNames, arm3, targetTimes, true);
	  qi::os::msleep(1000);
	  motion.angleInterpolation(jointNames, arm4, targetTimes, true);
	  motion.angleInterpolation(jointNames2, arm5, targetTimes2 , true);

	}
	if(BLACKAngle < -0.06){
	  BLACKAngle2 = BLACKAngle + 0.37;
	  Rarm2 = AL::ALValue::array(BLACKAngle2);  // "LShoulderRoll"
		    
	  motion.angleInterpolation(RjointNames2, Rarm1, targetTimes2 , true);
	  motion.angleInterpolation("RShoulderRoll", Rarm2, 0.6f, true);
	  motion.angleInterpolation(RjointNames, Rarm3, targetTimes, true);
	  qi::os::msleep(1000);
	  motion.angleInterpolation(RjointNames, Rarm4, targetTimes, true);
	  motion.angleInterpolation(RjointNames2, Rarm5, targetTimes2 , true);	  
	}	
	
	break;
	
      case 'l':   //ORANGE
	std::cout << "ORANGE selected" << std::endl;
	
	if (ORANGEAngle > 0.1){
	  ORANGEAngle2 = ORANGEAngle - 0.37;
	  arm2 = AL::ALValue::array(ORANGEAngle2);  // "LShoulderRoll"
		    
	  motion.angleInterpolation(jointNames2, arm1, targetTimes2 , true);
	  motion.angleInterpolation("LShoulderRoll", arm2, 0.6f, true);
	  motion.angleInterpolation(jointNames, arm3, targetTimes, true);
	  qi::os::msleep(1000);
	  motion.angleInterpolation(jointNames, arm4, targetTimes, true);
	  motion.angleInterpolation(jointNames2, arm5, targetTimes2 , true);
	}
	if (ORANGEAngle < 0.1 && ORANGEAngle > -0.06){
	  ORANGEAngle2 = -0.31f;//ORANGEAngle - 0.37;
	  ORANGEAngle3 = 0.31f;//ORANGEAngle + 0.37;
	  Barm2 = AL::ALValue::array(ORANGEAngle2, ORANGEAngle3);  // "LShoulderRoll"
		    
	  motion.angleInterpolation(BjointNames2, Barm1, BtargetTimes2 , true);
	  motion.angleInterpolation(jointNames3, Barm2, (0.6f, 0.6f), true);
	  motion.angleInterpolation(BjointNames, Barm3, BtargetTimes, true);
	  qi::os::msleep(1000);
	  motion.angleInterpolation(BjointNames, Barm4, BtargetTimes, true);
	  motion.angleInterpolation(BjointNames2, Barm5, BtargetTimes2 , true);

	}
	if(ORANGEAngle < -0.06){
	  ORANGEAngle2 = ORANGEAngle + 0.37;
	  Rarm2 = AL::ALValue::array(ORANGEAngle2);  // "LShoulderRoll"
		    
	  motion.angleInterpolation(RjointNames2, Rarm1, targetTimes2 , true);
	  motion.angleInterpolation("RShoulderRoll", Rarm2, 0.6f, true);
	  motion.angleInterpolation(RjointNames, Rarm3, targetTimes, true);
	  qi::os::msleep(1000);
	  motion.angleInterpolation(RjointNames, Rarm4, targetTimes, true);
	  motion.angleInterpolation(RjointNames2, Rarm5, targetTimes2 , true);	  
	}
	
	break;
	
      default:	
	std::cout << "WRONG COMMAND! TYPE AGAIN: " << std::endl;
	break;
    }
    
    
  }

  
  camProxy.unsubscribe(clientName);
  return 0;
}