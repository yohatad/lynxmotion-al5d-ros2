/*******************************************************************************************************************
*
*   Exercise 1: Implementation of the divide-and-conquer go-to-position algorithm (goto1) 
*               and the MIMO go-to-position algorithm (goto2)
*               for the iRobot Create 2 mobile robot
*
*   This is the interface file.
*   For documentation, please see the application file
*
*   David Vernon
*   1 November 2021
*
*   Audit Trail
*   -----------
*
*
*
*******************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath>  
#include <ctype.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "coro_common/service_call.hpp"
#include "coro_common/spin.hpp"
#include "coro_common/package_path.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/teleport_absolute.hpp" // for turtle1/teleport_absolute service
#include "turtlesim/srv/set_pen.hpp"           // for turtle1/set_pen service
#include "std_srvs/srv/empty.hpp"             // for reset and clear services
#include "geometry_msgs/msg/twist.hpp"        // For geometry_msgs::msg::Twist
#include "nav_msgs/msg/odometry.hpp"          // For nav_msgs::msg::Odometry
#include <iomanip>                      // for std::setprecision and std::fixed

using namespace std;

#define ROS_PACKAGE_NAME    "module3"

/* Node shared by the callbacks and the spin loops; created in main() */
extern rclcpp::Node::SharedPtr ros_node;
#define PI 3.14159

/***************************************************************************************************************************

   Definitions for reading locomotion parameter data 

****************************************************************************************************************************/

#define MAX_FILENAME_LENGTH 200
#define STRING_LENGTH       200
#define KEY_LENGTH           40
#define NUMBER_OF_KEYS       11
#define GOING                 0  // used to switch between angle_tolerance_going and angle_tolerance_orienting
#define ORIENTING             1

typedef char keyword[KEY_LENGTH];

struct locomotionParameterDataType {
   float position_tolerance;
   float angle_tolerance_orienting;
   float angle_tolerance_going;
   float position_gain_dq; 
   float angle_gain_dq; 
   float position_gain_mimo; 
   float angle_gain_mimo;
   float min_linear_velocity;                           // m/s       ... from calibration; less than this and the motors are not actuated
   float max_linear_velocity;                           // m/s       ... see "Commanding your Create" on  https://github.com/AutonomyLab/create_robot
   float min_angular_velocity;                          // radians/s ... from calibration; less than this and the motors are not actuated
   float max_angular_velocity;                          // radians/s ... see "Commanding your Create" on  https://github.com/AutonomyLab/create_robot
};


/* Callback function, executed each time a new message arrives on the odom topic */

/* Callback function, executed each time a new message arrives on the odom topic */
void odomMessageReceived(const nav_msgs::msg::Odometry& msg);

int  signnum(float x);

void readLocomotionParameterData(char filename[], struct locomotionParameterDataType *locomotionParameterData);

void findMinimumVelocities(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub, rclcpp::Rate & rate, float max_linear_velocity,  float max_angular_velocity);
void setOdometryPose(float x, float y, float z);
void goToPoseDQ     (float x, float y, float theta, locomotionParameterDataType locomotionParameterData, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub, rclcpp::Rate & rate);
void goToPoseMIMO1  (float x, float y, float theta, locomotionParameterDataType locomotionParameterData, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub, rclcpp::Rate & rate);

void display_error_and_exit(char error_message[]);
void prompt_and_exit(int status);
void prompt_and_continue();
void print_message_to_file(FILE *fp, char message[]);

void odomMessageReceived(const nav_msgs::msg::Odometry& msg);

int  signnum(float x);
void readLocomotionParameterData(char filename[]);

void display_error_and_exit(char error_message[]);
void prompt_and_exit(int status);
void prompt_and_continue();
void print_message_to_file(FILE *fp, char message[]);
