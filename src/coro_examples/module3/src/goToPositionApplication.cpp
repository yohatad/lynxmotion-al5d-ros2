/*******************************************************************************************************************
*
*  Module 3 example: Implementation of the divide-and-conquer go-to-position controller  
*
*  The program reads an input file goToPositionInput.txt.
*  This file contains a sequence of commands, one command per line.
*  
*  Each command comprises seven fields:
*  a key string  (either "goto1" or "goto2") followed by six floating point numbers.
*  "goto1" invokes the divide-and-conquer algorithm
*  "goto2" invokes the MIMO controller (not implemented)
*  
*  The first three numbers give the start pose of the turtle (x, y, theta, respectively).
*  
*  The second three numbers give the goal pose of the turtle (x, y, theta, respectively).
*
*  For each command, the turtle is positioned at the start pose and then drives to goal pose
*  using the specified algorithm.
*
*  The current turtle pose is sensed by subscribing to the turtle1/pose topic.
*  The turtle is driven by publishing velocity commands on the turtle1/cmd_vel topic.
*
*   Sample Input
*   goto1 2.0 1.0 3.14 8.0 8.0 0.0
*   goto2 2.0 1.0 3.14 8.0 9.0 0.0
*
*
*   David Vernon
*   10 March 2021
*
*   Audit Trail
*   -----------
* 
*
*******************************************************************************************************************/

#include <module3/goToPosition.h> 

/* global variables with the current turtle pose */

float                current_x     = 0; 
float                current_y     = 0; 
float                current_theta = 0;


/* All ROS objects live inside run(), so they are destroyed before the context is shut down */
static int run(int argc, char **argv) {
  
   bool debug = false;

   FILE                 *fp_in;                    
   std::string          packagedir;
   char                 path[MAX_FILENAME_LENGTH];
   char                 input_filename[MAX_FILENAME_LENGTH]            = "goToPositionInput.txt";
   char                 path_and_input_filename[MAX_FILENAME_LENGTH]   = "";
   int                  end_of_file;

   bool                 success = true;
   bool                 go_to_pose = false;
   geometry_msgs::msg::Twist msg; 

   float                start_x;
   float                start_y;
   float                start_theta;

   float                goal_x;
   float                goal_y;
   float                goal_theta;

   float                goal_direction;
   
   float                position_error;
   float                angle_error;

   float                delta_pos       = 0.5;   // positional tolerance 0.2
   float                delta_theta     = 0.05;

   float                kp_pos1         = 0.5;
   float                kp_theta1       = 1.0;

   float                kp_pos2         = 0.2;   // 0.2
   float                kp_theta2       = 1.0;

   float                publish_rate     = 50;   // rate at which cmd_vel commands are published
   
   char                 command[10];
   

   /* Initialize the ROS system and become a node */

   rclcpp::init(argc, argv);                       // Initialize the ROS system
   ros_node = rclcpp::Node::make_shared("module3"); // Become a node


   /* Create a subscriber object for pose */

   auto sub = ros_node->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 1000, &poseMessageReceived);


   /* Create a publisher object for velocity commands */

   auto pub = ros_node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1000);
   rclcpp::Rate rate(publish_rate); // Publish  at this rate (in Hz)  until the node is shut down


   /* Create client objects for the required services */

   auto teleportClient = ros_node->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
   auto setpenClient   = ros_node->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
   auto resetClient    = ros_node->create_client<std_srvs::srv::Empty>("reset");
   auto clearClient    = ros_node->create_client<std_srvs::srv::Empty>("clear");

   if (!coro_common::waitForService(ros_node, teleportClient) ||
       !coro_common::waitForService(ros_node, setpenClient)   ||
       !coro_common::waitForService(ros_node, resetClient)    ||
       !coro_common::waitForService(ros_node, clearClient)) {
      return 0;   // waitForService only gives up when ROS is shut down, i.e. the user pressed Ctrl-C
   }


   /* Create the request objects for the teleport, set_pen, reset and clear services */

   auto teleport_arguments = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>(); // reposition the turtle without locomotion
   auto pen_arguments      = std::make_shared<turtlesim::srv::SetPen::Request>();           // turn the pen on/off and change colour

   auto reset_arguments    = std::make_shared<std_srvs::srv::Empty::Request>();             // to return to the default configuration

   auto clear_arguments    = std::make_shared<std_srvs::srv::Empty::Request>();             // to clear the background


   
  /* construct the full path and filename */
   
   packagedir = coro_common::packagePath(ROS_PACKAGE_NAME); // get the package directory
 
   if (debug) cout << "Package directory: " << packagedir << endl;

   strcat(path_and_input_filename, packagedir.c_str());  
   strcat(path_and_input_filename, "/data/"); 
   strcat(path_and_input_filename, input_filename);

   if (debug) printf("Input file is  %s\n",path_and_input_filename);

   /* open the input file */
   
   if ((fp_in = fopen(path_and_input_filename,"r")) == 0) {
      printf("Error: can't open %s\n",path_and_input_filename);
      prompt_and_exit(1);
   }
 
   end_of_file=fscanf(fp_in, "%s %f %f %f %f %f %f", command, &start_x, &start_y, &start_theta,
			                                      &goal_x,  &goal_y,  &goal_theta);

   while (end_of_file != EOF) {

      if (debug) {
         printf("Input data: %s %f %f %f %f %f %f\n", command, start_x, start_y, start_theta,
	                                                       goal_x,  goal_y,  goal_theta);
      }


      /* reset the simulator and return the turtle to the default pose */

      /*
      success = coro_common::callService(ros_node, resetClient, reset_arguments) != nullptr;

      if (!success) {
         RCLCPP_ERROR_STREAM(ros_node->get_logger(), "Turtle failed to reset" );
      }
      */

      /* clear the simulator */

      success = coro_common::callService(ros_node, clearClient, clear_arguments) != nullptr;

      if (!success) {
         RCLCPP_ERROR_STREAM(ros_node->get_logger(), "Turtle failed to clear" );
      }

      /* turn the pen off so that we don't see a trace when the turtle teleports */

      pen_arguments->off = 1;
      
      success = coro_common::callService(ros_node, setpenClient, pen_arguments) != nullptr;
      
      if (!success) {
      	 RCLCPP_ERROR_STREAM(ros_node->get_logger(), "TurtlePen failed to switch off");
      }
      
      /* move the turtle to the start pose by teleporting */

      /* version 1 */
      /*
      req.x = start_x;  // fill in the request data members
      req.y = start_y;
      req.theta = start_theta;
     
      success = teleportClient.call(req, resp);

      if (!success) {
         RCLCPP_ERROR_STREAM(ros_node->get_logger(), "Turtle failed to teleport" );
      }
      */
      
      teleport_arguments->x     = start_x;
      teleport_arguments->y     = start_y;
      teleport_arguments->theta = start_theta;

      success = coro_common::callService(ros_node, teleportClient, teleport_arguments) != nullptr;
      
      if (!success) {
         RCLCPP_ERROR_STREAM(ros_node->get_logger(), "Turtle failed to teleport" );
      }

      current_x     = start_x;
      current_y     = start_y;
      current_theta = start_theta;
      
      /* turn the pen again so that we do see a trace when the turtle moves */
      
      pen_arguments->off    = 0;
      pen_arguments->r      = 255;
      pen_arguments->g      = 255;
      pen_arguments->b      = 255;
      pen_arguments->width = 1;
      
      success = coro_common::callService(ros_node, setpenClient, pen_arguments) != nullptr;
      if (!success) {
      	 RCLCPP_ERROR_STREAM(ros_node->get_logger(), "TurtlePen failed to switch off");
      }

      /* now execute the command to drive the turtlebot to the goal pose */
      
      if (strcmp(command, "goto1")==0) {

 	 /* divide and conquer algorithm */
	
	 do {

	    /* get the current pose */

	    coro_common::spinOnce(ros_node);      // Let ROS take over to handle the callback

	    if (debug) {
	      printf("Current pose: %f %f %f\n",current_x, current_y, current_theta);
	    }

	    position_error = sqrt((goal_x - current_x)*(goal_x - current_x) +
			          (goal_y - current_y)*(goal_y - current_y));

	    if (debug) {
	      printf("Position error: %f\n",position_error);
	    }

	    goal_direction = atan2((goal_y - current_y),(goal_x - current_x));
	    angle_error = goal_direction - current_theta;

	    if (fabs(angle_error) > delta_theta) {
               msg.linear.x  = 0;          
               msg.angular.z = kp_theta1 * angle_error;
	    }
	    else {
	       msg.linear.x  = kp_pos1 * position_error;          
               msg.angular.z = 0;
	    }

	    	
	    if (go_to_pose) {

	       /* if the turtlebot has reached the destination and     */
               /* if the flag is set to achieve the goal pose then     */
	       /* adjust the orientation to match the goal orientation */

	      if (position_error < delta_pos) {
                  angle_error = goal_theta - current_theta;
	          msg.linear.x  = 0;          
                  msg.angular.z = kp_theta1 * angle_error;
               }
	    }
	    
            pub->publish(msg);              // Publish the message

            if (debug) {
               RCLCPP_INFO_STREAM(ros_node->get_logger(), "velocity command" << " linear ="  << msg.linear.x
   	                                          << " angular =" << msg.angular.z);
            }
	    
            rate.sleep(); // Wait until it's time for another iteration
	    
	 } while((position_error >= delta_pos) && rclcpp::ok());
      }
      else {

	/* MIMO algorithm */

      }
      
      /* prompt user to continue */

      prompt_and_continue();

      end_of_file=fscanf(fp_in, "%s %f %f %f %f %f %f", command, &start_x, &start_y, &start_theta,
			                                         &goal_x,  &goal_y,  &goal_theta);
   }
   return 0;
}

int main(int argc, char **argv) {
   int status = 1;
   try {
      status = run(argc, argv);
   }
   catch (const std::exception &e) {
      /* Ctrl-C shuts the ROS context down underneath a running ROS call: that is a normal exit */
      if (rclcpp::ok()) {
         fprintf(stderr, "Fatal error: %s\n", e.what());
      }
      else {
         status = 0;
      }
   }
   ros_node.reset();     // release the node before the ROS context goes away
   rclcpp::shutdown();
   return status;
}
