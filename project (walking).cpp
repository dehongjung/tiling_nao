// DYROS - Dynamic Robotic System Lab


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

Mat img_rgb,img_gray,canny_output,drawing;
int thresh = 100;
int max_thresh = 255;
int DARKBLUEflag = 0, BLACKflag = 0, YELLOWflag = 0, WHITEflag = 0, ORANGEflag = 0, WALKflag = 0, PXflag = 0;
float horizontal_pixels = 0;
float x=0, y=0, theta=0;


void *PrintHello(void *threadid)
{
  AL::ALTextToSpeechProxy fTtsProxy("192.168.1.1", 9559);
  
   long tid;
   tid = (long)threadid;
   cout << "Running the thread " << tid << endl;
   fTtsProxy.say("Running the thread");
   
   //pthread_exit(NULL);
}

void *Walk(void *threadid)
{
  AL::ALMotionProxy motion("192.168.1.1", 9559);
  AL::ALRobotPostureProxy postureProxy("192.168.1.1", 9559);
  
  //struct thread_data *my_data;
  
  //my_data = (struct thread_data *) threadarg;
  
  long distance;
  distance = (long)threadid;
   float theta = 0;
   
   
   if(WALKflag == 0){
      //// Enable arms control by Walk algorithm
      motion.setWalkArmsEnabled(true, true);

      motion.setMotionConfig(AL::ALValue::array(
	AL::ALValue::array("ENABLE_FOOT_CONTACT_PROTECTION", true)));
      
      float x     = 0.04f;
      float y     = 0.0f;

	theta = (horizontal_pixels*0.4f)/120.0f;
 
      //theta = AL::Math::PI/2.0f;
      motion.moveTo(x, y, 0.0f);
      //proxy->waitUntilMoveIsFinished();
    } 
    if(WALKflag == 1){      
      x     = 0.0f;
      y     = 0.0f;
      motion.moveTo(x, y, 0.0f);
      //proxy->waitUntilMoveIsFinished();
    }
    if(WALKflag == 2){      
      x     = 0.0f;
      y     = 0.0f;
      motion.moveTo(x, y, 0.0f);
      //proxy->waitUntilMoveIsFinished();
    }
    if(WALKflag == 3){      
      x     = (0.7*distance)/100;   //Converted Centimeters to Meters and Multiplied by 0.7 according to experimental variance (robot walked 6.5 cm instead of 5 cm)
      y     = 0.0f;

	theta = (horizontal_pixels*0.4f)/120.0f;

      motion.moveTo(x, y, 0.0f);
      motion.waitUntilMoveIsFinished();
      postureProxy.goToPosture("Crouch", 0.7);
      WALKflag = 4;
      qi::os::msleep(2000);
      //motion.stiffnessInterpolation("Body", 0.0f, 1.0f);
      //pthread_exit(NULL);
      //break;
      
    }
   
   //pthread_exit(NULL);
}


int main(int argc, char* argv[]) {
  static int DARKBLUEposX, DARKBLUEposY, DARKBLUElastX, DARKBLUElastY; //To hold the X and Y position of tracking color object
  static int ORANGEposX, ORANGEposY, ORANGElastX, ORANGElastY, distance;
  static int WHITEposX, WHITEposY, WHITElastX, WHITElastY;
  static int BLACKposX, BLACKposY, BLACKlastX, BLACKlastY;
  static int YELLOWposX, YELLOWposY, YELLOWlastX, YELLOWlastY;
  int i, j;
  //struct thread_data td;
  
  
  

  
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
  AL::ALValue jointNames = AL::ALValue::array("Body");
  float stiffnessLists = 1.0f;
  float timeLists      = 1.0f;
  motion.stiffnessInterpolation(jointNames, stiffnessLists, timeLists);
  //motion.moveInit();
  
  
  //motion.angleInterpolationWithSpeed("HeadPitch", 0.5f, 0.6f);
  
  
  const std::string robotIp(argv[1]);
  
  CvMemStorage* storage = cvCreateMemStorage(0);
  /** Create a proxy to ALVideoDevice on the robot.*/
  ALVideoDeviceProxy camProxy(robotIp, 9559);

  /** Subscribe a client image requiring 320*240 and BGR colorspace.*/
  const std::string clientName = camProxy.subscribeCamera("testsss2", kBottomCamera, kQVGA, kBGRColorSpace, 30);
  
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
  
  while ((char) cv::waitKey(30) != 27) {
    ALValue img = camProxy.getImageRemote(clientName);
    imgHeader.data = (uchar*) img[6].GetBinary();
    
    pthread_create(&threads, NULL, Walk, (void *)1);
    
    cvtColor(imgHeader,img_gray,CV_RGB2GRAY);
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
    
    inRange(ORANGETrackingFrames, Scalar(0,0,194), Scalar(151, 166, 255),ORANGETrackingFrames); //ORANGE
    inRange(BLACKTrackingFrames, Scalar(0,0,0), Scalar(87, 84, 87),BLACKTrackingFrames);  //BLACK
    inRange(DARKBLUETrackingFrames, Scalar(109,0,0), Scalar(255, 68, 118),DARKBLUETrackingFrames);  //DARK BLUE
    inRange(YELLOWTrackingFrames, Scalar(22,230,0), Scalar(190, 255, 255),YELLOWTrackingFrames);  //YELLOW
    inRange(WHITETrackingFrames, Scalar(223,231,169), Scalar(255, 255, 255),WHITETrackingFrames);  //WHITE
    
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
    
    pthread_create(&threads, NULL, Walk, (void *)distance);
    
    printf("Distance: %d cm\n", distance);
    printf("Y Axis: %d px\n", WHITElastY);
    
    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    Canny( img_gray, canny_output, thresh, thresh*2, 3 );
    imshow("Canny",canny_output);
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    
    vector<Point> approxTriangle;
    
    
    if (DARKBLUEposX > 0 && DARKBLUEposY > 0 && DARKBLUElastX > 0 && DARKBLUElastY >> 0) {
      
      line(*scribbleFrame, cvPoint(DARKBLUEposX, DARKBLUEposY), cvPoint(DARKBLUElastX, DARKBLUElastY),
	   cvScalar(0, 255, 255), 1); //To draw a continuous stretch of lines
      line(imgHeader, cvPoint(DARKBLUEposX, DARKBLUEposY), cvPoint(DARKBLUElastX, DARKBLUElastY),
	   cvScalar(0, 255, 255), 5); //To draw a yello point on the center of the colored object
      //cvPoint is used to create a Point data type which holds the pixel location
      
      
      for(size_t i = 0; i < contours.size(); i++){
	approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);
	if(approxTriangle.size() == 4){
	  if(contourArea(contours[i]) > 300){
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
	    break;
	  }
	}
      }
      /*if (DARKBLUEflag>2){
	
	    WALKflag++;
	    DARKBLUEflag = 0;
	    
      }*/
      
    }
    else if (ORANGEposX > 0 && ORANGEposY > 0 && ORANGElastX > 0 && ORANGElastY >> 0) {
      
      line(*scribbleFrame, cvPoint(ORANGEposX, ORANGEposY), cvPoint(ORANGElastX, ORANGElastY),
	   cvScalar(0, 255, 255), 1); //To draw a continuous stretch of lines
      line(imgHeader, cvPoint(ORANGEposX, ORANGEposY), cvPoint(ORANGElastX, ORANGElastY),
	   cvScalar(0, 255, 255), 5); //To draw a yello point on the center of the colored object
      //cvPoint is used to create a Point data type which holds the pixel location
      
      for(size_t i = 0; i < contours.size(); i++){
	approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);
	if(approxTriangle.size() == 4){
	  if(contourArea(contours[i]) > 200){
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
	    break;
	  }
	}
      }
      /*if(GREENflag > 2){

	    
	    GREENflag = 0;
      }*/
      
    }
    else if (WHITEposX > 0 && WHITEposY > 0 && WHITElastX > 0 && WHITElastY >> 0) {
      
      
      line(*scribbleFrame, cvPoint(WHITEposX, WHITEposY), cvPoint(WHITElastX, WHITElastY),
	   cvScalar(0, 255, 255), 1); //To draw a continuous stretch of lines
      line(imgHeader, cvPoint(WHITEposX, WHITEposY), cvPoint(WHITElastX, WHITElastY),
	   cvScalar(0, 255, 255), 5); //To draw a yello point on the center of the colored object
      //cvPoint is used to create a Point data type which holds the pixel location
      
      for(size_t i = 0; i < contours.size(); i++){
	approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);
	if(approxTriangle.size() == 4){
	  if(contourArea(contours[i]) > 200){
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
      if (WHITEflag>2){
	
	    WALKflag++;
	    WHITEflag = 0;
	    
      }
    }
    else if (BLACKposX > 0 && BLACKposY > 0 && BLACKlastX > 0 && BLACKlastY >> 0) {
      
      
      line(*scribbleFrame, cvPoint(BLACKposX, BLACKposY), cvPoint(BLACKlastX, BLACKlastY),
	   cvScalar(0, 255, 255), 1); //To draw a continuous stretch of lines
      line(imgHeader, cvPoint(BLACKposX, BLACKposY), cvPoint(BLACKlastX, BLACKlastY),
	   cvScalar(0, 255, 255), 5); //To draw a yello point on the center of the colored object
      //cvPoint is used to create a Point data type which holds the pixel location
      
      for(size_t i = 0; i < contours.size(); i++){
	approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);
	if(approxTriangle.size() == 4){
	  if(contourArea(contours[i]) > 200){
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
	    break;
	  }
	}
      }
    }
    else if (YELLOWposX > 0 && YELLOWposY > 0 && YELLOWlastX > 0 && YELLOWlastY >> 0) {
      
      
      line(*scribbleFrame, cvPoint(YELLOWposX, YELLOWposY), cvPoint(YELLOWlastX, YELLOWlastY),
	   cvScalar(0, 255, 255), 1); //To draw a continuous stretch of lines
      line(imgHeader, cvPoint(YELLOWposX, YELLOWposY), cvPoint(YELLOWlastX, YELLOWlastY),
	   cvScalar(0, 255, 255), 5); //To draw a yello point on the center of the colored object
      //cvPoint is used to create a Point data type which holds the pixel location
      
      for(size_t i = 0; i < contours.size(); i++){
	approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);
	if(approxTriangle.size() == 4){
	  if(contourArea(contours[i]) > 200){
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
    }
    
      /*if(BLUEflag > 2){

	    
	    BLUEflag = 0;
      }
      
    }else{
      DARKBLUEflag = 0;
      GREENflag = 0;
      BLUEflag = 0;
    }*/

    imshow("Squares",drawing);

    
    imshow(objWindow, DARKBLUETrackingFrames);
    imshow(objWindow, WHITETrackingFrames);
    imshow(objWindow, ORANGETrackingFrames);
    imshow(objWindow, BLACKTrackingFrames);
    imshow(objWindow, YELLOWTrackingFrames);
    
    camProxy.releaseImage(clientName);
    
    //imshow(scribbleWindow, *scribbleFrame);
    imshow(mouseController, imgHeader);
    //imshow(objWindow, colorTrackingFrames);
    add(imgHeader, *scribbleFrame, resutantFrame); //Add two Matrix of the same size
    imshow(resultWindow, resutantFrame);
    waitKey(1); // OpenCV way of adding a delay, generally used to get a Key info.
    if (WALKflag == 4)
      break;
  }
  
  camProxy.unsubscribe(clientName);
  return 0;
}