/*******************************************************************************************************************
*
*   Module 3 example: implementation of the divide-and-conquer go-to-position algorithm 
*
*   This is the interface file.
*   For documentation, please see the application file
*
*   David Vernon
*   10 March 2021
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
#include <iomanip>                      // for std::setprecision and std::fixed

using namespace std;

#define ROS_PACKAGE_NAME    "module3"

/* Node shared by the callbacks and the spin loops; created in main() */
extern rclcpp::Node::SharedPtr ros_node;
#define MAX_FILENAME_LENGTH 200


/* Callback function, executed each time a new pose message arrives */

void poseMessageReceived(const turtlesim::msg::Pose& msg);


void display_error_and_exit(char error_message[]);
void prompt_and_exit(int status);
void prompt_and_continue();
void print_message_to_file(FILE *fp, char message[]);
