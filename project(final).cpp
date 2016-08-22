/// DYROS - Dynamic Robotic System Lab



// Aldebaran includes.
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alproxies/altexttospeechproxy.h>
#include <alvalue/alvalue.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alsonarproxy.h>

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
float horizontal_pixels = 0, head = 0.0f, vertical_pixels = 0;
float x=0, y=0, theta=0;


void *Walk(void *threadid)
{
  AL::ALMotionProxy motion("192.168.1.1", 9559);
  AL::ALRobotPostureProxy postureProxy("192.168.1.1", 9559);
  ALVideoDeviceProxy camProxy("192.168.1.1", 9559);
  
  
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

      float x     = 1.0f;
      float y     = 0.0f;

      theta = (horizontal_pixels*0.7f)/120.0f;
      printf("THETA = %f\n", theta);
      printf("Thread Running1\n");
      motion.moveTo(x, y, -theta);
      
      //motion.angleInterpolation("HeadPitch", 0.0f + head, 0.6f, true);
      //head = head + 0.087f;
      
      //proxy->waitUntilMoveIsFinished();
    } 
    if(WALKflag == 1){      
      x     = 0.050f;
      y     = 0.0f;
      theta = (horizontal_pixels*0.3f)/120.0f;
      //proxy->waitUntilMoveIsFinished();
      /**Change Camera */      
      camProxy.setParam(kCameraSelectID,1);  //CHANGE TO THE BOTTOM CAMERA (0 - TOP, 1 - BOTTOM)
      
      motion.moveTo(x, y, theta); 
      printf("Thread Running 2\n");
      //postureProxy.goToPosture("Crouch", 0.7);
      motion.waitUntilMoveIsFinished();
      qi::os::msleep(3000);
      //motion.stiffnessInterpolation("Body", 0.0f, 1.0f);
      WALKflag = 2;
      
    }
    if(WALKflag == 2){      
      x     = 0.0f;
      y     = 0.0f;
      if(horizontal_pixels < -20)
	theta = -0.1f;
      if(horizontal_pixels > 16)
	theta = 0.1f;
      if(horizontal_pixels >= -20 && horizontal_pixels <= 16){
	theta = 0.0f;
	WALKflag = 5;
      }
      motion.moveTo(x, y, theta);
     
      

    }
    if(WALKflag == 3){      
      x     = (0.7*distance)/100;   //Converted Centimeters to Meters and Multiplied by 0.7 according to experimental variance (robot walked 6.5 cm instead of 5 cm)
      y     = 0.0f;

	theta = (horizontal_pixels*0.4f)/120.0f;

      //motion.moveTo(x, y, 0.0f);
      //motion.waitUntilMoveIsFinished();
      //postureProxy.goToPosture("Crouch", 0.7);
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
  
  //--------------------------------------------------------------------------
  
  std::vector<std::string> names;
  AL::ALValue times, keys;
  names.reserve(26);
  times.arraySetSize(26);
  keys.arraySetSize(26);

  names.push_back("HeadPitch");
  times[0].arraySetSize(7);
  keys[0].arraySetSize(7);

  times[0][0] = 0.640000;
  keys[0][0] = 0.00302604;
  times[0][1] = 1.56000;
  keys[0][1] = 0.00916204;
  times[0][2] = 2.80000;
  keys[0][2] = 0.00916204;
  times[0][3] = 4.32000;
  keys[0][3] = 0.00916204;
  times[0][4] = 6.04000;
  keys[0][4] = 0.00916204;
  times[0][5] = 7.44000;
  keys[0][5] = 0.0199000;
  times[0][6] = 9.00000;
  keys[0][6] = 0.0214340;

  names.push_back("HeadYaw");
  times[1].arraySetSize(7);
  keys[1].arraySetSize(7);

  /*times[1][0] = 0.640000;
  keys[1][0] = 0.157960;
  times[1][1] = 1.56000;
  keys[1][1] = 0.168698;
  times[1][2] = 2.80000;
  keys[1][2] = 0.170232;
  times[1][3] = 4.32000;
  keys[1][3] = 0.165630;
  times[1][4] = 6.04000;
  keys[1][4] = 0.168698;
  times[1][5] = 7.44000;
  keys[1][5] = 0.161028;
  times[1][6] = 9.00000;
  keys[1][6] = 0.156426;*/
  
  times[1][0] = 0.640000;
  keys[1][0] = 0.0;
  times[1][1] = 1.56000;
  keys[1][1] = 0.0;
  times[1][2] = 2.80000;
  keys[1][2] = 0.0;
  times[1][3] = 4.32000;
  keys[1][3] = 0.0;
  times[1][4] = 6.04000;
  keys[1][4] = 0.0;
  times[1][5] = 7.44000;
  keys[1][5] = 0.0;
  times[1][6] = 9.00000;
  keys[1][6] = 0.0;
 
  names.push_back("LAnklePitch");
  times[2].arraySetSize(7);
  keys[2].arraySetSize(7);

  times[2][0] = 0.640000;
  keys[2][0] = -0.423426;
  times[2][1] = 1.56000;
  keys[2][1] = -1.07384;
  times[2][2] = 2.80000;
  keys[2][2] = 0.0766580;
  times[2][3] = 4.32000;
  keys[2][3] = 0.467828;
  times[2][4] = 6.04000;
  keys[2][4] = 0.463226;
  times[2][5] = 7.44000;
  keys[2][5] = 0.654976;
  times[2][6] = 9.00000;
  keys[2][6] = 0.793036;

  names.push_back("LAnkleRoll");
  times[3].arraySetSize(7);
  keys[3].arraySetSize(7);

  times[3][0] = 0.640000;
  keys[3][0] = 0.00771196;
  times[3][1] = 1.56000;
  keys[3][1] = 0.110490;
  times[3][2] = 2.80000;
  keys[3][2] = 0.0107800;
  times[3][3] = 4.32000;
  keys[3][3] = -0.0137640;
  times[3][4] = 6.04000;
  keys[3][4] = -0.0122300;
  times[3][5] = 7.44000;
  keys[3][5] = 0.0123140;
  times[3][6] = 9.00000;
  keys[3][6] = -0.0321720;

  names.push_back("LElbowRoll");
  times[4].arraySetSize(7);
  keys[4].arraySetSize(7);

  times[4][0] = 0.640000;
  keys[4][0] = -0.288350;
  times[4][1] = 1.56000;
  keys[4][1] = -0.702530;
  times[4][2] = 2.80000;
  keys[4][2] = -0.719404;
  times[4][3] = 4.32000;
  keys[4][3] = -0.730142;
  times[4][4] = 6.04000;
  keys[4][4] = -0.730142;
  times[4][5] = 7.44000;
  keys[4][5] = -0.730142;
  times[4][6] = 9.00000;
  keys[4][6] = -0.745482;

  names.push_back("LElbowYaw");
  times[5].arraySetSize(7);
  keys[5].arraySetSize(7);

  times[5][0] = 0.640000;
  keys[5][0] = -0.219404;
  times[5][1] = 1.56000;
  keys[5][1] = -1.85465;
  times[5][2] = 2.80000;
  keys[5][2] = -1.86078;
  times[5][3] = 4.32000;
  keys[5][3] = -1.87152;
  times[5][4] = 6.04000;
  keys[5][4] = -1.86845;
  times[5][5] = 7.44000;
  keys[5][5] = -1.86232;
  times[5][6] = 9.00000;
  keys[5][6] = -1.86845;

  names.push_back("LHand");
  times[6].arraySetSize(7);
  keys[6].arraySetSize(7);

  times[6][0] = 0.640000;
  keys[6][0] = 0.00685565;
  times[6][1] = 1.56000;
  keys[6][1] = 0.00685565;
  times[6][2] = 2.80000;
  keys[6][2] = 0.00685565;
  times[6][3] = 4.32000;
  keys[6][3] = 0.00684867;
  times[6][4] = 6.04000;
  keys[6][4] = 0.00684867;
  times[6][5] = 7.44000;
  keys[6][5] = 0.00684867;
  times[6][6] = 9.00000;
  keys[6][6] = 0.00684867;

  names.push_back("LHipPitch");
  times[7].arraySetSize(7);
  keys[7].arraySetSize(7);

  times[7][0] = 0.640000;
  keys[7][0] = -0.300622;
  times[7][1] = 1.56000;
  keys[7][1] = -0.926494;
  times[7][2] = 2.80000;
  keys[7][2] = -1.27318;
  times[7][3] = 4.32000;
  keys[7][3] = -0.952572;
  times[7][4] = 6.04000;
  keys[7][4] = -0.952572;
  times[7][5] = 7.44000;
  keys[7][5] = -0.866668;
  times[7][6] = 9.00000;
  keys[7][6] = -1.24250;

  names.push_back("LHipRoll");
  times[8].arraySetSize(7);
  keys[8].arraySetSize(7);

  times[8][0] = 0.640000;
  keys[8][0] = -0.0260360;
  times[8][1] = 1.56000;
  keys[8][1] = -0.133416;
  times[8][2] = 2.80000;
  keys[8][2] = 0.0383920;
  times[8][3] = 4.32000;
  keys[8][3] = 0.242414;
  times[8][4] = 6.04000;
  keys[8][4] = 0.239346;
  times[8][5] = 7.44000;
  keys[8][5] = 0.0307220;
  times[8][6] = 9.00000;
  keys[8][6] = -0.374254;

  names.push_back("LHipYawPitch");
  times[9].arraySetSize(7);
  keys[9].arraySetSize(7);

  times[9][0] = 0.640000;
  keys[9][0] = -0.185572;
  times[9][1] = 1.56000;
  keys[9][1] = -0.148756;
  times[9][2] = 2.80000;
  keys[9][2] = -0.480100;
  times[9][3] = 4.32000;
  keys[9][3] = -0.969446;
  times[9][4] = 6.04000;
  keys[9][4] = -0.970980;
  times[9][5] = 7.44000;
  keys[9][5] = -0.977116;
  times[9][6] = 9.00000;
  keys[9][6] = -1.05228;

  names.push_back("LKneePitch");
  times[10].arraySetSize(7);
  keys[10].arraySetSize(7);

  times[10][0] = 0.640000;
  keys[10][0] = 0.868202;
  times[10][1] = 1.56000;
  keys[10][1] = 2.11255;
  times[10][2] = 2.80000;
  keys[10][2] = 1.57231;
  times[10][3] = 4.32000;
  keys[10][3] = 1.09677;
  times[10][4] = 6.04000;
  keys[10][4] = 1.09677;
  times[10][5] = 7.44000;
  keys[10][5] = 0.748550;
  times[10][6] = 9.00000;
  keys[10][6] = 0.624296;

  names.push_back("LShoulderPitch");
  times[11].arraySetSize(7);
  keys[11].arraySetSize(7);

  times[11][0] = 0.640000;
  keys[11][0] = 1.52629;
  times[11][1] = 1.56000;
  keys[11][1] = 1.56924;
  times[11][2] = 2.80000;
  keys[11][2] = 1.54930;
  times[11][3] = 4.32000;
  keys[11][3] = 1.54470;
  times[11][4] = 6.04000;
  keys[11][4] = 1.54163;
  times[11][5] = 7.44000;
  keys[11][5] = 1.51555;
  times[11][6] = 9.00000;
  keys[11][6] = 1.50635;

  names.push_back("LShoulderRoll");
  times[12].arraySetSize(7);
  keys[12].arraySetSize(7);

  times[12][0] = 0.640000;
  keys[12][0] = 0.345108;
  times[12][1] = 1.56000;
  keys[12][1] = 0.345108;
  times[12][2] = 2.80000;
  keys[12][2] = 0.366584;
  times[12][3] = 4.32000;
  keys[12][3] = 0.388060;
  times[12][4] = 6.04000;
  keys[12][4] = 0.388060;
  times[12][5] = 7.44000;
  keys[12][5] = 0.381924;
  times[12][6] = 9.00000;
  keys[12][6] = 0.397264;

  names.push_back("LWristYaw");
  times[13].arraySetSize(7);
  keys[13].arraySetSize(7);

  times[13][0] = 0.640000;
  keys[13][0] = -1.50950;
  times[13][1] = 1.56000;
  keys[13][1] = -0.0399260;
  times[13][2] = 2.80000;
  keys[13][2] = -0.0383920;
  times[13][3] = 4.32000;
  keys[13][3] = -0.0414600;
  times[13][4] = 6.04000;
  keys[13][4] = -0.0414600;
  times[13][5] = 7.44000;
  keys[13][5] = -0.0414600;
  times[13][6] = 9.00000;
  keys[13][6] = -0.0414600;

  names.push_back("RAnklePitch");
  times[14].arraySetSize(7);
  keys[14].arraySetSize(7);

  times[14][0] = 0.640000;
  keys[14][0] = -0.516916;
  times[14][1] = 1.56000;
  keys[14][1] = -1.18630;
  times[14][2] = 2.80000;
  keys[14][2] = -1.18630;
  times[14][3] = 4.32000;
  keys[14][3] = -1.18574;
  times[14][4] = 6.04000;
  keys[14][4] = -1.18574;
  times[14][5] = 7.44000;
  keys[14][5] = -1.14586;
  times[14][6] = 9.00000;
  keys[14][6] = -0.630432;

  names.push_back("RAnkleRoll");
  times[15].arraySetSize(7);
  keys[15].arraySetSize(7);

  times[15][0] = 0.640000;
  keys[15][0] = 0.00310996;
  times[15][1] = 1.56000;
  keys[15][1] = -0.0398420;
  times[15][2] = 2.80000;
  keys[15][2] = -0.0766580;
  times[15][3] = 4.32000;
  keys[15][3] = -0.0781920;
  times[15][4] = 6.04000;
  keys[15][4] = -0.0766580;
  times[15][5] = 7.44000;
  keys[15][5] = -0.0137640;
  times[15][6] = 9.00000;
  keys[15][6] = 0.397761;

  names.push_back("RElbowRoll");
  times[16].arraySetSize(7);
  keys[16].arraySetSize(7);

  times[16][0] = 0.640000;
  keys[16][0] = 0.286900;
  times[16][1] = 1.56000;
  keys[16][1] = 0.793120;
  times[16][2] = 2.80000;
  keys[16][2] = 0.793120;
  times[16][3] = 4.32000;
  keys[16][3] = 0.793120;
  times[16][4] = 6.04000;
  keys[16][4] = 0.909704;
  times[16][5] = 7.44000;
  keys[16][5] = 0.862150;
  times[16][6] = 9.00000;
  keys[16][6] = 0.205598;

  names.push_back("RElbowYaw");
  times[17].arraySetSize(7);
  keys[17].arraySetSize(7);

  times[17][0] = 0.640000;
  keys[17][0] = -0.181054;
  times[17][1] = 1.56000;
  keys[17][1] = -0.420358;
  times[17][2] = 2.80000;
  keys[17][2] = -0.420358;
  times[17][3] = 4.32000;
  keys[17][3] = -0.420358;
  times[17][4] = 6.04000;
  keys[17][4] = -0.937316;
  times[17][5] = 7.44000;
  keys[17][5] = -1.02629;
  times[17][6] = 9.00000;
  keys[17][6] = -0.0368580;

  names.push_back("RHand");
  times[18].arraySetSize(7);
  keys[18].arraySetSize(7);

  times[18][0] = 0.640000;
  keys[18][0] = 0.0104091;
  times[18][1] = 1.56000;
  keys[18][1] = 0.0104091;
  times[18][2] = 2.80000;
  keys[18][2] = 0.0104091;
  times[18][3] = 4.32000;
  keys[18][3] = 0.0104091;
  times[18][4] = 6.04000;
  keys[18][4] = 0.0104091;
  times[18][5] = 7.44000;
  keys[18][5] = 0.0104091;
  times[18][6] = 9.00000;
  keys[18][6] = 0.0103812;

  names.push_back("RHipPitch");
  times[19].arraySetSize(7);
  keys[19].arraySetSize(7);

  times[19][0] = 0.640000;
  keys[19][0] = -0.289968;
  times[19][1] = 1.56000;
  keys[19][1] = -0.806926;
  times[19][2] = 2.80000;
  keys[19][2] = -0.579894;
  times[19][3] = 4.32000;
  keys[19][3] = -0.300706;
  times[19][4] = 6.04000;
  keys[19][4] = -0.297638;
  times[19][5] = 7.44000;
  keys[19][5] = -0.538476;
  times[19][6] = 9.00000;
  keys[19][6] = -1.52944;

  names.push_back("RHipRoll");
  times[20].arraySetSize(7);
  keys[20].arraySetSize(7);

  times[20][0] = 0.640000;
  keys[20][0] = -0.0689880;
  times[20][1] = 1.56000;
  keys[20][1] = 0.0153820;
  times[20][2] = 2.80000;
  keys[20][2] = 0.0107800;
  times[20][3] = 4.32000;
  keys[20][3] = -0.121144;
  times[20][4] = 6.04000;
  keys[20][4] = -0.119610;
  times[20][5] = 7.44000;
  keys[20][5] = -0.134950;
  times[20][6] = 9.00000;
  keys[20][6] = -0.257670;

  names.push_back("RHipYawPitch");
  times[21].arraySetSize(7);
  keys[21].arraySetSize(7);

  times[21][0] = 0.640000;
  keys[21][0] = -0.185572;
  times[21][1] = 1.56000;
  keys[21][1] = -0.148756;
  times[21][2] = 2.80000;
  keys[21][2] = -0.480100;
  times[21][3] = 4.32000;
  keys[21][3] = -0.969446;
  times[21][4] = 6.04000;
  keys[21][4] = -0.970980;
  times[21][5] = 7.44000;
  keys[21][5] = -0.977116;
  times[21][6] = 9.00000;
  keys[21][6] = -1.05228;

  names.push_back("RKneePitch");
  times[22].arraySetSize(7);
  keys[22].arraySetSize(7);

  times[22][0] = 0.640000;
  keys[22][0] = 0.949588;
  times[22][1] = 1.56000;
  keys[22][1] = 2.11255;
  times[22][2] = 2.80000;
  keys[22][2] = 2.11255;
  times[22][3] = 4.32000;
  keys[22][3] = 2.11255;
  times[22][4] = 6.04000;
  keys[22][4] = 2.11255;
  times[22][5] = 7.44000;
  keys[22][5] = 2.11255;
  times[22][6] = 9.00000;
  keys[22][6] = 2.11255;

  names.push_back("RShoulderPitch");
  times[23].arraySetSize(7);
  keys[23].arraySetSize(7);

  times[23][0] = 0.640000;
  keys[23][0] = 1.39445;
  times[23][1] = 1.56000;
  keys[23][1] = 1.00174;
  times[23][2] = 2.80000;
  keys[23][2] = 0.994074;
  times[23][3] = 4.32000;
  keys[23][3] = 0.983336;
  times[23][4] = 6.04000;
  keys[23][4] = 0.208666;
  times[23][5] = 7.44000;
  keys[23][5] = 0.216336;
  times[23][6] = 9.00000;
  keys[23][6] = 0.374338;

  names.push_back("RShoulderRoll");
  times[24].arraySetSize(7);
  keys[24].arraySetSize(7);

  times[24][0] = 0.640000;
  keys[24][0] = -0.355930;
  times[24][1] = 1.56000;
  keys[24][1] = -0.929646;
  times[24][2] = 2.80000;
  keys[24][2] = -0.937316;
  times[24][3] = 4.32000;
  keys[24][3] = -0.937316;
  times[24][4] = 6.04000;
  keys[24][4] = -0.222472;
  times[24][5] = 7.44000;
  keys[24][5] = -0.334454;
  times[24][6] = 9.00000;
  keys[24][6] = -0.00157596;

  names.push_back("RWristYaw");
  times[25].arraySetSize(7);
  keys[25].arraySetSize(7);

  times[25][0] = 0.640000;
  keys[25][0] = 1.63674;
  times[25][1] = 1.56000;
  keys[25][1] = -0.182588;
  times[25][2] = 2.80000;
  keys[25][2] = -0.182588;
  times[25][3] = 4.32000;
  keys[25][3] = -0.182588;
  times[25][4] = 6.04000;
  keys[25][4] = -0.181054;
  times[25][5] = 7.44000;
  keys[25][5] = -0.181054;
  times[25][6] = 9.00000;
  keys[25][6] = -0.185656;
  //-------------------------------------------------------------
  
  
  std::vector<std::string> names2;
  AL::ALValue times2, keys2;
  names2.reserve(26);
  times2.arraySetSize(26);
  keys2.arraySetSize(26);

  names2.push_back("HeadPitch");
  times2[0].arraySetSize(6);
  keys2[0].arraySetSize(6);

  times2[0][0] = 0.280000;
  keys2[0][0] = 0.0214340;
  times2[0][1] = 3.24000;
  keys2[0][1] = 0.0199001;
  times2[0][2] = 4.96000;
  keys2[0][2] = 0.00916204;
  times2[0][3] = 6.64000;
  keys2[0][3] = 0.0214340;
  times2[0][4] = 8.48000;
  keys2[0][4] = 0.0229680;
  times2[0][5] = 10.8800;
  keys2[0][5] = 0.0229680;

  names2.push_back("HeadYaw");
  times2[1].arraySetSize(6);
  keys2[1].arraySetSize(6);

  times2[1][0] = 0.280000;
  keys2[1][0] = 0.156426;
  times2[1][1] = 3.24000;
  keys2[1][1] = 0.161028;
  times2[1][2] = 4.96000;
  keys2[1][2] = 0.168698;
  times2[1][3] = 6.64000;
  keys2[1][3] = 0.165630;
  times2[1][4] = 8.48000;
  keys2[1][4] = 0.165630;
  times2[1][5] = 10.8800;
  keys2[1][5] = 0.164096;

  names2.push_back("LAnklePitch");
  times2[2].arraySetSize(6);
  keys2[2].arraySetSize(6);

  times2[2][0] = 0.280000;
  keys2[2][0] = 0.793036;
  times2[2][1] = 3.24000;
  keys2[2][1] = 0.654977;
  times2[2][2] = 4.96000;
  keys2[2][2] = 0.463226;
  times2[2][3] = 6.64000;
  keys2[2][3] = 0.0797260;
  times2[2][4] = 8.48000;
  keys2[2][4] = -1.07077;
  times2[2][5] = 10.8800;
  keys2[2][5] = -0.424960;

  names2.push_back("LAnkleRoll");
  times2[3].arraySetSize(6);
  keys2[3].arraySetSize(6);

  times2[3][0] = 0.280000;
  keys2[3][0] = -0.0321720;
  times2[3][1] = 3.24000;
  keys2[3][1] = 0.0123140;
  times2[3][2] = 4.96000;
  keys2[3][2] = -0.0122300;
  times2[3][3] = 6.64000;
  keys2[3][3] = 0.0107800;
  times2[3][4] = 8.48000;
  keys2[3][4] = 0.105888;
  times2[3][5] = 10.8800;
  keys2[3][5] = 0.00924596;

  names2.push_back("LElbowRoll");
  times2[4].arraySetSize(6);
  keys2[4].arraySetSize(6);

  times2[4][0] = 0.280000;
  keys2[4][0] = -0.745483;
  times2[4][1] = 3.24000;
  keys2[4][1] = -0.730143;
  times2[4][2] = 4.96000;
  keys2[4][2] = -0.730143;
  times2[4][3] = 6.64000;
  keys2[4][3] = -0.693326;
  times2[4][4] = 8.48000;
  keys2[4][4] = -0.694860;
  times2[4][5] = 10.8800;
  keys2[4][5] = -0.335904;

  names2.push_back("LElbowYaw");
  times2[5].arraySetSize(6);
  keys2[5].arraySetSize(6);

  times2[5][0] = 0.280000;
  keys2[5][0] = -1.86846;
  times2[5][1] = 3.24000;
  keys2[5][1] = -1.86232;
  times2[5][2] = 4.96000;
  keys2[5][2] = -1.86846;
  times2[5][3] = 6.64000;
  keys2[5][3] = -1.86078;
  times2[5][4] = 8.48000;
  keys2[5][4] = -1.85772;
  times2[5][5] = 10.8800;
  keys2[5][5] = -0.227074;

  names2.push_back("LHand");
  times2[6].arraySetSize(6);
  keys2[6].arraySetSize(6);

  times2[6][0] = 0.280000;
  keys2[6][0] = 0.00684867;
  times2[6][1] = 3.24000;
  keys2[6][1] = 0.00684867;
  times2[6][2] = 4.96000;
  keys2[6][2] = 0.00684867;
  times2[6][3] = 6.64000;
  keys2[6][3] = 0.00683471;
  times2[6][4] = 8.48000;
  keys2[6][4] = 0.00683471;
  times2[6][5] = 10.8800;
  keys2[6][5] = 0.00683471;

  names2.push_back("LHipPitch");
  times2[7].arraySetSize(6);
  keys2[7].arraySetSize(6);

  times2[7][0] = 0.280000;
  keys2[7][0] = -1.24250;
  times2[7][1] = 3.24000;
  keys2[7][1] = -0.866668;
  times2[7][2] = 4.96000;
  keys2[7][2] = -0.952573;
  times2[7][3] = 6.64000;
  keys2[7][3] = -1.27164;
  times2[7][4] = 8.48000;
  keys2[7][4] = -0.924960;
  times2[7][5] = 10.8800;
  keys2[7][5] = -0.300622;

  names2.push_back("LHipRoll");
  times2[8].arraySetSize(6);
  keys2[8].arraySetSize(6);

  times2[8][0] = 0.280000;
  keys2[8][0] = -0.374254;
  times2[8][1] = 3.24000;
  keys2[8][1] = 0.0307220;
  times2[8][2] = 4.96000;
  keys2[8][2] = 0.239346;
  times2[8][3] = 6.64000;
  keys2[8][3] = 0.0429940;
  times2[8][4] = 8.48000;
  keys2[8][4] = -0.122678;
  times2[8][5] = 10.8800;
  keys2[8][5] = -0.0597840;

  names2.push_back("LHipYawPitch");
  times2[9].arraySetSize(6);
  keys2[9].arraySetSize(6);

  times2[9][0] = 0.280000;
  keys2[9][0] = -1.05228;
  times2[9][1] = 3.24000;
  keys2[9][1] = -0.977116;
  times2[9][2] = 4.96000;
  keys2[9][2] = -0.970981;
  times2[9][3] = 6.64000;
  keys2[9][3] = -0.480100;
  times2[9][4] = 8.48000;
  keys2[9][4] = -0.154892;
  times2[9][5] = 10.8800;
  keys2[9][5] = -0.185572;

  names2.push_back("LKneePitch");
  times2[10].arraySetSize(6);
  keys2[10].arraySetSize(6);

  times2[10][0] = 0.280000;
  keys2[10][0] = 0.624296;
  times2[10][1] = 3.24000;
  keys2[10][1] = 0.748551;
  times2[10][2] = 4.96000;
  keys2[10][2] = 1.09677;
  times2[10][3] = 6.64000;
  keys2[10][3] = 1.57538;
  times2[10][4] = 8.48000;
  keys2[10][4] = 2.11255;
  times2[10][5] = 10.8800;
  keys2[10][5] = 0.868202;

  names2.push_back("LShoulderPitch");
  times2[11].arraySetSize(6);
  keys2[11].arraySetSize(6);

  times2[11][0] = 0.280000;
  keys2[11][0] = 1.50635;
  times2[11][1] = 3.24000;
  keys2[11][1] = 1.51555;
  times2[11][2] = 4.96000;
  keys2[11][2] = 1.54163;
  times2[11][3] = 6.64000;
  keys2[11][3] = 1.55697;
  times2[11][4] = 8.48000;
  keys2[11][4] = 1.57384;
  times2[11][5] = 10.8800;
  keys2[11][5] = 1.49101;

  names2.push_back("LShoulderRoll");
  times2[12].arraySetSize(6);
  keys2[12].arraySetSize(6);

  times2[12][0] = 0.280000;
  keys2[12][0] = 0.397265;
  times2[12][1] = 3.24000;
  keys2[12][1] = 0.381923;
  times2[12][2] = 4.96000;
  keys2[12][2] = 0.388060;
  times2[12][3] = 6.64000;
  keys2[12][3] = 0.354312;
  times2[12][4] = 8.48000;
  keys2[12][4] = 0.337438;
  times2[12][5] = 10.8800;
  keys2[12][5] = 0.337438;

  names2.push_back("LWristYaw");
  times2[13].arraySetSize(6);
  keys2[13].arraySetSize(6);

  times2[13][0] = 0.280000;
  keys2[13][0] = -0.0414599;
  times2[13][1] = 3.24000;
  keys2[13][1] = -0.0414599;
  times2[13][2] = 4.96000;
  keys2[13][2] = -0.0414599;
  times2[13][3] = 6.64000;
  keys2[13][3] = -0.0583340;
  times2[13][4] = 8.48000;
  keys2[13][4] = -0.0568000;
  times2[13][5] = 10.8800;
  keys2[13][5] = -1.43587;

  names2.push_back("RAnklePitch");
  times2[14].arraySetSize(6);
  keys2[14].arraySetSize(6);

  times2[14][0] = 0.280000;
  keys2[14][0] = -0.630432;
  times2[14][1] = 3.24000;
  keys2[14][1] = -1.14586;
  times2[14][2] = 4.96000;
  keys2[14][2] = -1.18574;
  times2[14][3] = 6.64000;
  keys2[14][3] = -1.18574;
  times2[14][4] = 8.48000;
  keys2[14][4] = -1.18630;
  times2[14][5] = 10.8800;
  keys2[14][5] = -0.524586;

  names2.push_back("RAnkleRoll");
  times2[15].arraySetSize(6);
  keys2[15].arraySetSize(6);

  times2[15][0] = 0.280000;
  keys2[15][0] = 0.397761;
  times2[15][1] = 3.24000;
  keys2[15][1] = -0.0137640;
  times2[15][2] = 4.96000;
  keys2[15][2] = -0.0766580;
  times2[15][3] = 6.64000;
  keys2[15][3] = -0.0766580;
  times2[15][4] = 8.48000;
  keys2[15][4] = -0.0367740;
  times2[15][5] = 10.8800;
  keys2[15][5] = 0.00310996;

  names2.push_back("RElbowRoll");
  times2[16].arraySetSize(6);
  keys2[16].arraySetSize(6);

  times2[16][0] = 0.280000;
  keys2[16][0] = 0.205598;
  times2[16][1] = 3.24000;
  keys2[16][1] = 0.862151;
  times2[16][2] = 4.96000;
  keys2[16][2] = 0.909704;
  times2[16][3] = 6.64000;
  keys2[16][3] = 0.918908;
  times2[16][4] = 8.48000;
  keys2[16][4] = 0.934248;
  times2[16][5] = 10.8800;
  keys2[16][5] = 0.914306;

  names2.push_back("RElbowYaw");
  times2[17].arraySetSize(6);
  keys2[17].arraySetSize(6);

  times2[17][0] = 0.280000;
  keys2[17][0] = -0.0368580;
  times2[17][1] = 3.24000;
  keys2[17][1] = -1.02629;
  times2[17][2] = 4.96000;
  keys2[17][2] = -0.937317;
  times2[17][3] = 6.64000;
  keys2[17][3] = -1.18276;
  times2[17][4] = 8.48000;
  keys2[17][4] = -1.17355;
  times2[17][5] = 10.8800;
  keys2[17][5] = -1.20577;

  names2.push_back("RHand");
  times2[18].arraySetSize(6);
  keys2[18].arraySetSize(6);

  times2[18][0] = 0.280000;
  keys2[18][0] = 0.0103812;
  times2[18][1] = 3.24000;
  keys2[18][1] = 0.0104091;
  times2[18][2] = 4.96000;
  keys2[18][2] = 0.0104091;
  times2[18][3] = 6.64000;
  keys2[18][3] = 0.0103882;
  times2[18][4] = 8.48000;
  keys2[18][4] = 0.0103882;
  times2[18][5] = 10.8800;
  keys2[18][5] = 0.0103882;

  names2.push_back("RHipPitch");
  times2[19].arraySetSize(6);
  keys2[19].arraySetSize(6);

  times2[19][0] = 0.280000;
  keys2[19][0] = -1.52944;
  times2[19][1] = 3.24000;
  keys2[19][1] = -0.538476;
  times2[19][2] = 4.96000;
  keys2[19][2] = -0.297638;
  times2[19][3] = 6.64000;
  keys2[19][3] = -0.581428;
  times2[19][4] = 8.48000;
  keys2[19][4] = -0.811528;
  times2[19][5] = 10.8800;
  keys2[19][5] = -0.286900;

  names2.push_back("RHipRoll");
  times2[20].arraySetSize(6);
  keys2[20].arraySetSize(6);

  times2[20][0] = 0.280000;
  keys2[20][0] = -0.257670;
  times2[20][1] = 3.24000;
  keys2[20][1] = -0.134950;
  times2[20][2] = 4.96000;
  keys2[20][2] = -0.119610;
  times2[20][3] = 6.64000;
  keys2[20][3] = 0.0107800;
  times2[20][4] = 8.48000;
  keys2[20][4] = 0.0153820;
  times2[20][5] = 10.8800;
  keys2[20][5] = -0.0459780;

  names2.push_back("RHipYawPitch");
  times2[21].arraySetSize(6);
  keys2[21].arraySetSize(6);

  times2[21][0] = 0.280000;
  keys2[21][0] = -1.05228;
  times2[21][1] = 3.24000;
  keys2[21][1] = -0.977116;
  times2[21][2] = 4.96000;
  keys2[21][2] = -0.970981;
  times2[21][3] = 6.64000;
  keys2[21][3] = -0.480100;
  times2[21][4] = 8.48000;
  keys2[21][4] = -0.154892;
  times2[21][5] = 10.8800;
  keys2[21][5] = -0.185572;

  names2.push_back("RKneePitch");
  times2[22].arraySetSize(6);
  keys2[22].arraySetSize(6);

  times2[22][0] = 0.280000;
  keys2[22][0] = 2.11255;
  times2[22][1] = 3.24000;
  keys2[22][1] = 2.11255;
  times2[22][2] = 4.96000;
  keys2[22][2] = 2.11255;
  times2[22][3] = 6.64000;
  keys2[22][3] = 2.11255;
  times2[22][4] = 8.48000;
  keys2[22][4] = 2.11255;
  times2[22][5] = 10.8800;
  keys2[22][5] = 0.952656;

  names2.push_back("RShoulderPitch");
  times2[23].arraySetSize(6);
  keys2[23].arraySetSize(6);

  times2[23][0] = 0.280000;
  keys2[23][0] = 0.374338;
  times2[23][1] = 3.24000;
  keys2[23][1] = 0.216335;
  times2[23][2] = 4.96000;
  keys2[23][2] = 0.208666;
  times2[23][3] = 6.64000;
  keys2[23][3] = 0.240880;
  times2[23][4] = 8.48000;
  keys2[23][4] = 0.230142;
  times2[23][5] = 10.8800;
  keys2[23][5] = 0.366668;

  names2.push_back("RShoulderRoll");
  times2[24].arraySetSize(6);
  keys2[24].arraySetSize(6);

  times2[24][0] = 0.280000;
  keys2[24][0] = -0.00157596;
  times2[24][1] = 3.24000;
  keys2[24][1] = -0.334454;
  times2[24][2] = 4.96000;
  keys2[24][2] = -0.222472;
  times2[24][3] = 6.64000;
  keys2[24][3] = -0.213268;
  times2[24][4] = 8.48000;
  keys2[24][4] = -0.191792;
  times2[24][5] = 10.8800;
  keys2[24][5] = -0.188724;

  names2.push_back("RWristYaw");
  times2[25].arraySetSize(6);
  keys2[25].arraySetSize(6);

  times2[25][0] = 0.280000;
  keys2[25][0] = -0.185656;
  times2[25][1] = 3.24000;
  keys2[25][1] = -0.181053;
  times2[25][2] = 4.96000;
  keys2[25][2] = -0.181053;
  times2[25][3] = 6.64000;
  keys2[25][3] = -0.185656;
  times2[25][4] = 8.48000;
  keys2[25][4] = -0.185656;
  times2[25][5] = 10.8800;
  keys2[25][5] = -0.159578;
  
  
  
  
std::vector<std::string> names3;
AL::ALValue times3, keys3;
names3.reserve(26);
times3.arraySetSize(26);
keys3.arraySetSize(26);

names3.push_back("HeadPitch");
times3[0].arraySetSize(7);
keys3[0].arraySetSize(7);

times3[0][0] = 0.640000;
keys3[0][0] = 0.0168320;
times3[0][1] = 1.56000;
keys3[0][1] = 0.458624;
times3[0][2] = 2.80000;
keys3[0][2] = 0.231591;
times3[0][3] = 4.32000;
keys3[0][3] = 0.00916204;
times3[0][4] = 6.04000;
keys3[0][4] = 0.00916204;
times3[0][5] = 7.36000;
keys3[0][5] = 0.0199001;
times3[0][6] = 9.00000;
keys3[0][6] = 0.0214340;

names3.push_back("HeadYaw");
times3[1].arraySetSize(7);
keys3[1].arraySetSize(7);

times3[1][0] = 0.640000;
keys3[1][0] = 0.165630;
times3[1][1] = 1.56000;
keys3[1][1] = 0.0122300;
times3[1][2] = 2.80000;
keys3[1][2] = 0.00302603;
times3[1][3] = 4.32000;
keys3[1][3] = 0.165630;
times3[1][4] = 6.04000;
keys3[1][4] = 0.168698;
times3[1][5] = 7.36000;
keys3[1][5] = 0.161028;
times3[1][6] = 9.00000;
keys3[1][6] = 0.156426;

names3.push_back("LAnklePitch");
times3[2].arraySetSize(7);
keys3[2].arraySetSize(7);

times3[2][0] = 0.640000;
keys3[2][0] = -0.426494;
times3[2][1] = 1.56000;
keys3[2][1] = -1.07384;
times3[2][2] = 2.80000;
keys3[2][2] = 0.0781920;
times3[2][3] = 4.32000;
keys3[2][3] = 0.467829;
times3[2][4] = 6.04000;
keys3[2][4] = 0.463226;
times3[2][5] = 7.36000;
keys3[2][5] = 0.654977;
times3[2][6] = 9.00000;
keys3[2][6] = 0.793036;

names3.push_back("LAnkleRoll");
times3[3].arraySetSize(7);
keys3[3].arraySetSize(7);

times3[3][0] = 0.640000;
keys3[3][0] = 0.00771196;
times3[3][1] = 1.56000;
keys3[3][1] = 0.107422;
times3[3][2] = 2.80000;
keys3[3][2] = 0.0107800;
times3[3][3] = 4.32000;
keys3[3][3] = -0.0137640;
times3[3][4] = 6.04000;
keys3[3][4] = -0.0122300;
times3[3][5] = 7.36000;
keys3[3][5] = 0.0123140;
times3[3][6] = 9.00000;
keys3[3][6] = -0.0321720;

names3.push_back("LElbowRoll");
times3[4].arraySetSize(7);
keys3[4].arraySetSize(7);

times3[4][0] = 0.640000;
keys3[4][0] = -0.332836;
times3[4][1] = 1.56000;
keys3[4][1] = -0.661111;
times3[4][2] = 2.80000;
keys3[4][2] = -0.661111;
times3[4][3] = 4.32000;
keys3[4][3] = -0.730143;
times3[4][4] = 6.04000;
keys3[4][4] = -0.730143;
times3[4][5] = 7.36000;
keys3[4][5] = -0.730143;
times3[4][6] = 9.00000;
keys3[4][6] = -0.745483;

names3.push_back("LElbowYaw");
times3[5].arraySetSize(7);
keys3[5].arraySetSize(7);

times3[5][0] = 0.640000;
keys3[5][0] = -0.222472;
times3[5][1] = 1.56000;
keys3[5][1] = -1.84698;
times3[5][2] = 2.80000;
keys3[5][2] = -1.84851;
times3[5][3] = 4.32000;
keys3[5][3] = -1.87152;
times3[5][4] = 6.04000;
keys3[5][4] = -1.86846;
times3[5][5] = 7.36000;
keys3[5][5] = -1.86232;
times3[5][6] = 9.00000;
keys3[5][6] = -1.86846;

names3.push_back("LHand");
times3[6].arraySetSize(7);
keys3[6].arraySetSize(7);

times3[6][0] = 0.640000;
keys3[6][0] = 0.00682773;
times3[6][1] = 1.56000;
keys3[6][1] = 0.00683471;
times3[6][2] = 2.80000;
keys3[6][2] = 0.00684169;
times3[6][3] = 4.32000;
keys3[6][3] = 0.00684867;
times3[6][4] = 6.04000;
keys3[6][4] = 0.00684867;
times3[6][5] = 7.36000;
keys3[6][5] = 0.00684867;
times3[6][6] = 9.00000;
keys3[6][6] = 0.00684867;

names3.push_back("LHipPitch");
times3[7].arraySetSize(7);
keys3[7].arraySetSize(7);

times3[7][0] = 0.640000;
keys3[7][0] = -0.299088;
times3[7][1] = 1.56000;
keys3[7][1] = -0.926494;
times3[7][2] = 2.80000;
keys3[7][2] = -1.27164;
times3[7][3] = 4.32000;
keys3[7][3] = -0.952573;
times3[7][4] = 6.04000;
keys3[7][4] = -0.952573;
times3[7][5] = 7.36000;
keys3[7][5] = -0.866668;
times3[7][6] = 9.00000;
keys3[7][6] = -1.24250;

names3.push_back("LHipRoll");
times3[8].arraySetSize(7);
keys3[8].arraySetSize(7);

times3[8][0] = 0.640000;
keys3[8][0] = -0.0567160;
times3[8][1] = 1.56000;
keys3[8][1] = -0.116542;
times3[8][2] = 2.80000;
keys3[8][2] = 0.0383920;
times3[8][3] = 4.32000;
keys3[8][3] = 0.242414;
times3[8][4] = 6.04000;
keys3[8][4] = 0.239346;
times3[8][5] = 7.36000;
keys3[8][5] = 0.0307220;
times3[8][6] = 9.00000;
keys3[8][6] = -0.374254;

names3.push_back("LHipYawPitch");
times3[9].arraySetSize(7);
keys3[9].arraySetSize(7);

times3[9][0] = 0.640000;
keys3[9][0] = -0.185572;
times3[9][1] = 1.56000;
keys3[9][1] = -0.150290;
times3[9][2] = 2.80000;
keys3[9][2] = -0.480100;
times3[9][3] = 4.32000;
keys3[9][3] = -0.969447;
times3[9][4] = 6.04000;
keys3[9][4] = -0.970981;
times3[9][5] = 7.36000;
keys3[9][5] = -0.977116;
times3[9][6] = 9.00000;
keys3[9][6] = -1.05228;

names3.push_back("LKneePitch");
times3[10].arraySetSize(7);
keys3[10].arraySetSize(7);

times3[10][0] = 0.640000;
keys3[10][0] = 0.868202;
times3[10][1] = 1.56000;
keys3[10][1] = 2.11228;
times3[10][2] = 2.80000;
keys3[10][2] = 1.57844;
times3[10][3] = 4.32000;
keys3[10][3] = 1.09677;
times3[10][4] = 6.04000;
keys3[10][4] = 1.09677;
times3[10][5] = 7.36000;
keys3[10][5] = 0.748551;
times3[10][6] = 9.00000;
keys3[10][6] = 0.624296;

names3.push_back("LShoulderPitch");
times3[11].arraySetSize(7);
keys3[11].arraySetSize(7);

times3[11][0] = 0.640000;
keys3[11][0] = 1.49407;
times3[11][1] = 1.56000;
keys3[11][1] = 1.58151;
times3[11][2] = 2.80000;
keys3[11][2] = 1.57691;
times3[11][3] = 4.32000;
keys3[11][3] = 1.54470;
times3[11][4] = 6.04000;
keys3[11][4] = 1.54163;
times3[11][5] = 7.36000;
keys3[11][5] = 1.51555;
times3[11][6] = 9.00000;
keys3[11][6] = 1.50635;

names3.push_back("LShoulderRoll");
times3[12].arraySetSize(7);
keys3[12].arraySetSize(7);

times3[12][0] = 0.640000;
keys3[12][0] = 0.342041;
times3[12][1] = 1.56000;
keys3[12][1] = 0.329768;
times3[12][2] = 2.80000;
keys3[12][2] = 0.328234;
times3[12][3] = 4.32000;
keys3[12][3] = 0.388060;
times3[12][4] = 6.04000;
keys3[12][4] = 0.388060;
times3[12][5] = 7.36000;
keys3[12][5] = 0.381923;
times3[12][6] = 9.00000;
keys3[12][6] = 0.397265;

names3.push_back("LWristYaw");
times3[13].arraySetSize(7);
keys3[13].arraySetSize(7);

times3[13][0] = 0.640000;
keys3[13][0] = -1.43740;
times3[13][1] = 1.56000;
keys3[13][1] = -0.0598679;
times3[13][2] = 2.80000;
keys3[13][2] = -0.0568000;
times3[13][3] = 4.32000;
keys3[13][3] = -0.0414599;
times3[13][4] = 6.04000;
keys3[13][4] = -0.0414599;
times3[13][5] = 7.36000;
keys3[13][5] = -0.0414599;
times3[13][6] = 9.00000;
keys3[13][6] = -0.0414599;

names3.push_back("RAnklePitch");
times3[14].arraySetSize(7);
keys3[14].arraySetSize(7);

times3[14][0] = 0.640000;
keys3[14][0] = -0.521518;
times3[14][1] = 1.56000;
keys3[14][1] = -1.18574;
times3[14][2] = 2.80000;
keys3[14][2] = -1.18421;
times3[14][3] = 4.32000;
keys3[14][3] = -1.18574;
times3[14][4] = 6.04000;
keys3[14][4] = -1.18574;
times3[14][5] = 7.36000;
keys3[14][5] = -1.14586;
times3[14][6] = 9.00000;
keys3[14][6] = -0.630432;

names3.push_back("RAnkleRoll");
times3[15].arraySetSize(7);
keys3[15].arraySetSize(7);

times3[15][0] = 0.640000;
keys3[15][0] = 0.00310997;
times3[15][1] = 1.56000;
keys3[15][1] = -0.0398420;
times3[15][2] = 2.80000;
keys3[15][2] = -0.0766580;
times3[15][3] = 4.32000;
keys3[15][3] = -0.0781920;
times3[15][4] = 6.04000;
keys3[15][4] = -0.0766580;
times3[15][5] = 7.36000;
keys3[15][5] = -0.0137640;
times3[15][6] = 9.00000;
keys3[15][6] = 0.397761;

names3.push_back("RElbowRoll");
times3[16].arraySetSize(7);
keys3[16].arraySetSize(7);

times3[16][0] = 0.640000;
keys3[16][0] = 0.895898;
times3[16][1] = 1.56000;
keys3[16][1] = 0.771643;
times3[16][2] = 2.80000;
keys3[16][2] = 0.780848;
times3[16][3] = 4.32000;
keys3[16][3] = 0.793120;
times3[16][4] = 6.04000;
keys3[16][4] = 0.909704;
times3[16][5] = 7.36000;
keys3[16][5] = 0.862151;
times3[16][6] = 9.00000;
keys3[16][6] = 0.205598;

names3.push_back("RElbowYaw");
times3[17].arraySetSize(7);
keys3[17].arraySetSize(7);

times3[17][0] = 0.640000;
keys3[17][0] = -1.10759;
times3[17][1] = 1.56000;
keys3[17][1] = -0.418823;
times3[17][2] = 2.80000;
keys3[17][2] = -0.418823;
times3[17][3] = 4.32000;
keys3[17][3] = -0.420357;
times3[17][4] = 6.04000;
keys3[17][4] = -0.937317;
times3[17][5] = 7.36000;
keys3[17][5] = -1.02629;
times3[17][6] = 9.00000;
keys3[17][6] = -0.0368580;

names3.push_back("RHand");
times3[18].arraySetSize(7);
keys3[18].arraySetSize(7);

times3[18][0] = 0.640000;
keys3[18][0] = 0.0103882;
times3[18][1] = 1.56000;
keys3[18][1] = 0.0103882;
times3[18][2] = 2.80000;
keys3[18][2] = 0.0103882;
times3[18][3] = 4.32000;
keys3[18][3] = 0.0104091;
times3[18][4] = 6.04000;
keys3[18][4] = 0.0104091;
times3[18][5] = 7.36000;
keys3[18][5] = 0.0104091;
times3[18][6] = 9.00000;
keys3[18][6] = 0.0103812;

names3.push_back("RHipPitch");
times3[19].arraySetSize(7);
keys3[19].arraySetSize(7);

times3[19][0] = 0.640000;
keys3[19][0] = -0.288433;
times3[19][1] = 1.56000;
keys3[19][1] = -0.817664;
times3[19][2] = 2.80000;
keys3[19][2] = -0.581429;
times3[19][3] = 4.32000;
keys3[19][3] = -0.300706;
times3[19][4] = 6.04000;
keys3[19][4] = -0.297638;
times3[19][5] = 7.36000;
keys3[19][5] = -0.538476;
times3[19][6] = 9.00000;
keys3[19][6] = -1.52944;

names3.push_back("RHipRoll");
times3[20].arraySetSize(7);
keys3[20].arraySetSize(7);

times3[20][0] = 0.640000;
keys3[20][0] = -0.0536481;
times3[20][1] = 1.56000;
keys3[20][1] = 0.0153820;
times3[20][2] = 2.80000;
keys3[20][2] = 0.0199840;
times3[20][3] = 4.32000;
keys3[20][3] = -0.121144;
times3[20][4] = 6.04000;
keys3[20][4] = -0.119610;
times3[20][5] = 7.36000;
keys3[20][5] = -0.134950;
times3[20][6] = 9.00000;
keys3[20][6] = -0.257670;

names3.push_back("RHipYawPitch");
times3[21].arraySetSize(7);
keys3[21].arraySetSize(7);

times3[21][0] = 0.640000;
keys3[21][0] = -0.185572;
times3[21][1] = 1.56000;
keys3[21][1] = -0.150290;
times3[21][2] = 2.80000;
keys3[21][2] = -0.480100;
times3[21][3] = 4.32000;
keys3[21][3] = -0.969447;
times3[21][4] = 6.04000;
keys3[21][4] = -0.970981;
times3[21][5] = 7.36000;
keys3[21][5] = -0.977116;
times3[21][6] = 9.00000;
keys3[21][6] = -1.05228;

names3.push_back("RKneePitch");
times3[22].arraySetSize(7);
keys3[22].arraySetSize(7);

times3[22][0] = 0.640000;
keys3[22][0] = 0.951122;
times3[22][1] = 1.56000;
keys3[22][1] = 2.11255;
times3[22][2] = 2.80000;
keys3[22][2] = 2.11255;
times3[22][3] = 4.32000;
keys3[22][3] = 2.11255;
times3[22][4] = 6.04000;
keys3[22][4] = 2.11255;
times3[22][5] = 7.36000;
keys3[22][5] = 2.11255;
times3[22][6] = 9.00000;
keys3[22][6] = 2.11255;

names3.push_back("RShoulderPitch");
times3[23].arraySetSize(7);
keys3[23].arraySetSize(7);

times3[23][0] = 0.640000;
keys3[23][0] = 0.346725;
times3[23][1] = 1.56000;
keys3[23][1] = 1.04316;
times3[23][2] = 2.80000;
keys3[23][2] = 1.03089;
times3[23][3] = 4.32000;
keys3[23][3] = 0.983336;
times3[23][4] = 6.04000;
keys3[23][4] = 0.208666;
times3[23][5] = 7.36000;
keys3[23][5] = 0.216335;
times3[23][6] = 9.00000;
keys3[23][6] = 0.374338;

names3.push_back("RShoulderRoll");
times3[24].arraySetSize(7);
keys3[24].arraySetSize(7);

times3[24][0] = 0.640000;
keys3[24][0] = -0.187190;
times3[24][1] = 1.56000;
keys3[24][1] = -0.863683;
times3[24][2] = 2.80000;
keys3[24][2] = -0.865217;
times3[24][3] = 4.32000;
keys3[24][3] = -0.937317;
times3[24][4] = 6.04000;
keys3[24][4] = -0.222472;
times3[24][5] = 7.36000;
keys3[24][5] = -0.334454;
times3[24][6] = 9.00000;
keys3[24][6] = -0.00157596;

names3.push_back("RWristYaw");
times3[25].arraySetSize(7);
keys3[25].arraySetSize(7);

times3[25][0] = 0.640000;
keys3[25][0] = -0.174919;
times3[25][1] = 1.56000;
keys3[25][1] = -0.156510;
times3[25][2] = 2.80000;
keys3[25][2] = -0.156510;
times3[25][3] = 4.32000;
keys3[25][3] = -0.182588;
times3[25][4] = 6.04000;
keys3[25][4] = -0.181053;
times3[25][5] = 7.36000;
keys3[25][5] = -0.181053;
times3[25][6] = 9.00000;
keys3[25][6] = -0.185656;
  //--------------------------------------------------------------------------
  

  AL::ALMemoryProxy fMemoryProxy("192.168.1.1", 9559);
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
  motion.moveInit();
  motion.angleInterpolation("HeadPitch", 0.30f, 0.6f, true);
  
  //motion.angleInterpolationWithSpeed("HeadPitch", 0.5f, 0.6f);
  
  
  const std::string robotIp(argv[1]);
  
  CvMemStorage* storage = cvCreateMemStorage(0);
  /** Create a proxy to ALVideoDevice on the robot.*/
  ALVideoDeviceProxy camProxy(robotIp, 9559);

  /** Subscribe a client image requiring 320*240 and BGR colorspace.*/
  const std::string clientName = camProxy.subscribeCamera("testsss2", kBottomCamera, kQVGA, kBGRColorSpace, 30);
  
  /**Change Camera */
  camProxy.setParam(kCameraSelectID,0);  //CHANGE TO THE BOTTOM CAMERA (0 - TOP, 1 - BOTTOM)
  

  
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
    
    //pthread_create(&threads, NULL, Walk, (void *)1);
    
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
    //inRange(DARKBLUETrackingFrames, Scalar(109,0,0), Scalar(255, 68, 118),DARKBLUETrackingFrames);  //DARK BLUE DIA
    inRange(DARKBLUETrackingFrames, Scalar(111,28,37), Scalar(171, 71, 95),DARKBLUETrackingFrames); //DARK BLUE Noite
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
  
    //horizontal_pixels = DARKBLUElastX - 120;
    vertical_pixels = WHITElastY;
    distance = -0.332*WHITElastY + 109.82;     //Collected some experimental data (measured the distance bewtween the robot and the square according to the pixel position on the camera image), plotted it and did a Linear Regreesssion to get the equation
    
    pthread_create(&threads, NULL, Walk, (void *)distance);
    
    printf("Distance: %d cm\n", distance);
    printf("Y Axis: %d px\n", DARKBLUElastY);
    printf("WALKflag: %d\n", WALKflag);
    
    if (WALKflag == 0 && DARKBLUElastY >190){
      WALKflag = 1;
     
    }
    else if (WALKflag == 0 && ORANGElastY >200 ){
      WALKflag = 1;
    }
    else if (WALKflag == 0 && BLACKlastY >200 ){
      WALKflag = 1;
    }
    
    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    Canny( img_gray, canny_output, thresh, thresh*2, 3 );
    //imshow("Canny",canny_output);
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    
    vector<Point> approxTriangle;
    
   
    if (DARKBLUEposX > 0 && DARKBLUEposY > 0 && DARKBLUElastX > 0 && DARKBLUElastY >> 0) {
      
      horizontal_pixels = -1*(DARKBLUElastX-160);
      
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
	    //fTtsProxy.say("One dark blue square detected");
	    printf("DARKBLUE Square detected!\n");
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
      
      horizontal_pixels = -1*(ORANGElastX-160);
      
      line(*scribbleFrame, cvPoint(ORANGEposX, ORANGEposY), cvPoint(ORANGElastX, ORANGElastY),
	   cvScalar(0, 255, 255), 1); //To draw a continuous stretch of lines
      line(imgHeader, cvPoint(ORANGEposX, ORANGEposY), cvPoint(ORANGElastX, ORANGElastY),
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
	    //fTtsProxy.say("One orange square detected");
	    printf("ORANGE Square detected!\n");
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
	    //fTtsProxy.say("One white square detected");
	    printf("WHITE Square detected!\n");
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
      /*if (WHITEflag>2){
	
	    WALKflag++;
	    WHITEflag = 0;
	    
      }*/
    }
    else if (BLACKposX > 0 && BLACKposY > 0 && BLACKlastX > 0 && BLACKlastY >> 0) {
      
      horizontal_pixels = -1*(BLACKlastX - 160);
      
      
      line(*scribbleFrame, cvPoint(BLACKposX, BLACKposY), cvPoint(BLACKlastX, BLACKlastY),
	   cvScalar(0, 255, 255), 1); //To draw a continuous stretch of lines
      line(imgHeader, cvPoint(BLACKposX, BLACKposY), cvPoint(BLACKlastX, BLACKlastY),
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
	    //fTtsProxy.say("One black square detected");
	    printf("BLACK Square detected!\n");
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
	    //fTtsProxy.say("One yellow square detected");
	    printf("YELLOW Square detected!\n");
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
    } //else
      //horizontal_pixels = 0.0f;
    
      /*if(BLUEflag > 2){

	    
	    BLUEflag = 0;
      }
      
    }else{
      DARKBLUEflag = 0;
      GREENflag = 0;
      BLUEflag = 0;
    }*/
    printf("HORIZONTAL: %f \n", horizontal_pixels);
    //imshow("Squares",drawing);

    
    /*imshow(objWindow, DARKBLUETrackingFrames);
    imshow(objWindow, WHITETrackingFrames);
    imshow(objWindow, ORANGETrackingFrames);
    imshow(objWindow, BLACKTrackingFrames);
    imshow(objWindow, YELLOWTrackingFrames);*/
    
    camProxy.releaseImage(clientName);
    
    //imshow(scribbleWindow, *scribbleFrame);
    //imshow(mouseController, imgHeader);
    //imshow(objWindow, colorTrackingFrames);
    add(imgHeader, *scribbleFrame, resutantFrame); //Add two Matrix of the same size
    imshow(resultWindow, resutantFrame);
    waitKey(1); // OpenCV way of adding a delay, generally used to get a Key info.
    if (WALKflag == 5){
      motion.moveTo(0.03f, 0.0f, 0.0f);
      qi::os::msleep(1000);
      motion.angleInterpolation(names, keys, times, true);
      qi::os::msleep(500);
      motion.angleInterpolation(names2, keys2, times2, true);
      qi::os::msleep(1000);
      motion.moveTo(0.4,0,0);
      qi::os::msleep(400);
      motion.angleInterpolation(names3, keys3, times3, true);
      qi::os::msleep(1000);
      motion.angleInterpolation(names2, keys2, times2, true);
      qi::os::msleep(1000);
      break;
    }
      
  }
  
  camProxy.unsubscribe(clientName);
  return 0;
}