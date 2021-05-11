#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <deque>

// Define a global client that can request services
ros::ServiceClient client;

static const float vel = 0.5;
static const std::pair<int, int> center_range(250, 550);
static const float ball_prox_thrsh = 0.45;

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
    bool ball_reached = false;
    uint32_t ball_center_x = 0;
    std::vector<uint32_t> x_coords;

    for(uint32_t i = 0; i < img.height; ++i)
    {
        for(uint32_t j = 0; j < img.width; ++j)
        {
            const auto& r = img.data[img.step * i + 3 * j];
            const auto& g = img.data[img.step * i + 3 * j + 1];
            const auto& b = img.data[img.step * i + 3 * j + 2];
            if(r == white_pixel && g == white_pixel && b == white_pixel)
            {
                ball_found = true;
                x_coords.push_back(j);
            }
        }
    }
    
    if(!x_coords.empty()) 
    {
        ball_center_x = std::accumulate(std::begin(x_coords),
            std::end(x_coords), 0.0) / x_coords.size();
        
        float ball_prox = static_cast<float>(x_coords.size()) / (img.width * img.height);
        if(ball_prox > ball_prox_thrsh)
        {
            ball_reached = true;
        }
    }

    if(!ball_found || ball_reached)
    {
        drive_robot(.0, .0);
    }
    else
    {
        if(ball_center_x >= 0 && ball_center_x < center_range.first)
        {
            drive_robot(0.0, vel);
        }
        else if(ball_center_x >= center_range.first && ball_center_x < center_range.second)
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
