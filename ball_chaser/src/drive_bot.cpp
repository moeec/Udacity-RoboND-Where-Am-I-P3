#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"  //Added: Include the ball_chaser "DriveToTarget" header file

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// Added: A handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints


bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
  auto linear_x = (float)req.linear_x;
  auto angular_z = (float)req.angular_z;

  ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_z:%1.2f", linear_x, angular_z);

  // Motor_command object of type geometry_msgs::Twist
  geometry_msgs::Twist motor_command;
  
  // Getting wheel velocity and yaw 
  motor_command.linear.x = linear_x;
  motor_command.angular.z = angular_z;
  
  // Publishing the requested velocities
  motor_command_publisher.publish(motor_command);

  // Message feedback returned with the requested wheel velocities
  res.msg_feedback = "Move command is set - linear_x: " + std::to_string(linear_x)
                                     + " , angular_z: " + std::to_string(angular_z);
  ROS_INFO_STREAM(res.msg_feedback);

  return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    ROS_INFO("Ready to send commands for move robot:");

    // Handle ROS comm. events

    ros::spin();

    return 0;
}
