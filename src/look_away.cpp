#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <cstdlib>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

// Define global vector of joints last position, moving state of the arm, and the client that can request services
std::vector<double> joints_LastPosition{ 0, 0 };
bool moving_State = false;
ros::ServiceClient client;

// This function calls the /safe_move service to safely move the arm to the center position
void move_arm_center()
{
    ROS_INFO_STREAM("Moving the arm to the center");

    // Request centered joint angles [1.57, 1.57]
    simple_arm::GoToPosition srv;
    srv.request.joint_1 = 1.57;
    srv.request.joint_2 = 1.57;

    // Call the safe_move service and pass the requested joint angles
    if (client.call(srv))
        ROS_INFO_STREAM(srv.response.msg_feedback);
    else
        ROS_ERROR("Failed to call service safe_move");
}

// This callback function continuously executes and reads the arm joint angles position
void joint_states_callback(const sensor_msgs::JointState js)
{
    // Get joints current position
    std::vector<double> joints_CurrentPosition = js.position;

    // Define a tolerance threshold to compare double values
    double tolerance = 0.0005;

    // Check if the arm is moving by comparing its current joints position to its latest
    if (fabs(joints_CurrentPosition[0] - joints_LastPosition[0]) < tolerance && fabs(joints_CurrentPosition[1] - joints_LastPosition[1]) < tolerance)
        moving_State = false;
    else {
        moving_State = true;
        joints_LastPosition = joints_CurrentPosition;
    }
}

// This callback function continuously executes and reads the image data
void look_away_callback(const sensor_msgs::Image img)
{
    // Set uniform_image as true, and define the uniform pixel value
    bool uniform_Image = true;
    int uniform_Pixel = 178;

    // Loop through each pixel in the image and check if its equal to the uniform pixel
    for (int i = 0; i < img.height * img.width; i++) {
        if (img.data[i] != uniform_Pixel) {
            uniform_Image = false;
            break;
        }
    }

    // If the image is uniform and the arm is not moving, move the arm to the center
    if (uniform_Image == true && moving_State == false)
        move_arm_center();
}

int main(int argc, char** argv)
{
    // Initialize the look_away node and crete a handle to it
    ros::init(argc, argv, "look_away");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from safe_move
    client = n.serviceClient<simple_arm::GoToPosition>("safe_move");

    // Subscribe to /simple_arm/joint_states topic to read the arm joints position inside the joint_states_callback function
    ros::Subscriber sub = n.subscribe("/simple_arm/joint_states", 1000, joint_states_callback);

    // Subscribe to rgb_camera/image_raw topic to read the image data inside the look_away_callback function
    ros::Subscriber sub2 = n.subscribe("rgb_camera/image_raw", 1000, look_away_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
