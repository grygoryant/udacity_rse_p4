#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

float vel = 1.0;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    if (!client.call(srv))
        ROS_ERROR("Failed to drive robot!");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    bool ball_found = false;
    uint32_t ball_region = 0;

    for(uint32_t i = 0; !ball_found && i < img.height; ++i)
    {
        for(uint32_t j = 0; j < img.width; ++j)
        {
            const auto& r = img.data[img.step * i + 3 * j];
	        const auto& g = img.data[img.step * i + 3 * j + 1];
	        const auto& b = img.data[img.step * i + 3 * j + 2];
	        if(r == white_pixel && g == white_pixel && b == white_pixel)
	        {
	            ball_found = true;
	            ball_region = j;
	            break;
	        }
        }
    }
 
    if(!ball_found)
    {
        drive_robot(.0, .0);
    }
    else
    {
        if(ball_region >= 0 && ball_region < 250)
        {
            drive_robot(0.0, vel);
        }
        else if(ball_region >= 250 && ball_region < 550)
        {
            drive_robot(vel, 0.0);
        }
        else
        {
            drive_robot(0.0, -vel);
        }
    }
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
