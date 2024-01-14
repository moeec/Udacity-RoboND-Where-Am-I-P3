#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
// DONE: Request a service and pass the velocities to it to drive the robot
  ROS_INFO("Sending request to move the robot.");

  // Initialize service with request values
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  // Attempt to call the service and log error if call fails
  if (!client.call(srv)) 
  {
    ROS_ERROR("Failed to call service /ball_chaser/command_robot");
  }
  
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image& img) 
    {
    enum Side {LEFT, RIGHT, MID, NO_BALL};
    Side side = NO_BALL;

    for (int i = 0; i < img.height * img.step; i += 3) 

        {
        if (img.data[i] == 255 && img.data[i+1] == 255 && img.data[i+2] == 255) { // white ball found
            auto col = i % img.step;
            side = (col < img.step * 0.4) ? LEFT : (col > img.step * 0.6) ? RIGHT : MID;
            break;
        }
    }

    float linear_x = 0.0, angular_z = 0.0;
    switch(side) 
    {
        case LEFT:   linear_x = 0.5; angular_z = 1.0; break;
        case RIGHT:  linear_x = 0.5; angular_z = -1.0; break;
        case MID:    linear_x = 0.5; break;
        case NO_BALL: break; // Default 0.0 values are already set for both speeds
    }

    drive_robot(linear_x, angular_z);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
