/* 
  Example use of openCV to perform camera calibration
  ---------------------------------------------------
 
  (This is the interface file: it contains the declarations of dedicated functions to implement the application.
  These function are called by client code in the application file. The functions are defined in the implementation file.)

  David Vernon
  10 June 2018
    
  Ported to OpenCV 4
  David Vernon
  11 July 2024
*/
 

#define ROS_PACKAGE_NAME "module5"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <ctype.h>
#include <iostream>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <time.h>


//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>



// Must be included after opencv2/opencv.hpp to avoid incompatiability
#include <ncurses.h>
#include "rclcpp/rclcpp.hpp"
#include "coro_common/package_path.hpp"



#include "std_msgs/msg/float64_multi_array.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"
#include "ros_gz_interfaces/srv/delete_entity.hpp"
#include "ros_gz_interfaces/msg/entity.hpp"
#include "coro_common/service_call.hpp"
#include "coro_common/spin.hpp"

// CV Bridge includes
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/image_encodings.hpp"


 

#define TRUE  1
#define FALSE 0
#define MAX_STRING_LENGTH 80
#define MAX_FILENAME_LENGTH 200
#define MAX_NUMBER_OF_CONTROL_POINTS 100 
#define CHECKERBOARD_MODEL_NAME "checkerboard"

using namespace std;
using namespace cv;

struct imagePointType {
   int u, v;
};


struct worldPointType {
   float x, y, z;
};

/* function prototypes go here */ 
int  getImageControlPoints(string configurationFilename, int numberOfViews, int *numberOfControlPoints, imagePointType imagePoints[],
                             FILE* fp_world_points, float cameraX, float cameraY, float boardZ);
void getSimulatedImageControlPoints(int numberOfControlPoints, float focalLength, worldPointType worldPoints[], imagePointType imagePoints[]);
void getSimulatedWorldControlPoints(int numberOfCornersWidth, int numberOfCornersHeight, int squareSize, 
                                    int numberOfViews,int originX, int originY, int originZ, 
                                    int *numberOfControlPoints, worldPointType worldPoints[]) ;
void prompt_and_exit(int status);
void prompt_and_continue();
int _kbhit();
void spawn_checkerboard(const char* sdf_filename, float x, float y, float z, float pitch, float yaw, float roll);
void imageMessageReceived(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
size_t fsize(FILE *fp);

/* Node used by every ROS call in this program; created in main() */
extern rclcpp::Node::SharedPtr ros_node;
void writeWorldCoordinatesToFile(FILE *fp_world_points, float cameraX, float cameraY, float boardZ, float boxsize, Size size);
void delete_checkerboard();
void deleteFiles(const char* data_dir);
