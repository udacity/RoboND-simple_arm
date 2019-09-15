#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

class LookAwayPubSub {
public:
  LookAwayPubSub():
    joints_last_position_{0,0},
    moving_state_(false) {
      // Define a client service capable of requesting services from safe_move
      client_ = n_.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");
      // Subscribe to /simple_arm/joint_states topic to read the arm joints position inside the joint_states_callback function
      sub1_ = n_.subscribe("/simple_arm/joint_states", 10, &LookAwayPubSub::joint_states_callback, this);
      // Subscribe to rgb_camera/image_raw topic to read the image data inside the look_away_callback function
      sub2_ = n_.subscribe("rgb_camera/image_raw", 10, &LookAwayPubSub::look_away_callback, this);
    }
  // This function calls the safe_move service to safely move the arm to the center position
  void move_arm_center()
  {
      ROS_INFO_STREAM("Moving the arm to the center");

      // Request centered joint angles [1.57, 1.57]
      simple_arm::GoToPosition srv;
      srv.request.joint_1 = 1.57;
      srv.request.joint_2 = 1.57;

      // Call the safe_move service and pass the requested joint angles
      if (!client_.call(srv))
          ROS_ERROR("Failed to call service safe_move");
  }

  // This callback function continuously executes and reads the arm joint angles position
  void joint_states_callback(const sensor_msgs::JointState js)
  {
      // Get joints current position
      std::vector<double> joints_current_position = js.position;

      // Define a tolerance threshold to compare double values
      double tolerance = 0.0005;

      // Check if the arm is moving by comparing its current joints position to its latest
      if (fabs(joints_current_position[0] - joints_last_position_[0]) < tolerance && fabs(joints_current_position[1] - joints_last_position_[1]) < tolerance)
          moving_state_ = false;
      else {
          moving_state_ = true;
          joints_last_position_ = joints_current_position;
      }
  }

  // This callback function continuously executes and reads the image data
  void look_away_callback(const sensor_msgs::Image img)
  {
      bool uniform_image = true;
      // Loop through each pixel in the image and check if its equal to the first one
      for (int i = 0; i < img.height * img.step; i++) {
          if (img.data[i] - img.data[0] != 0) {
              uniform_image = false;
              break;
          }
      }
      // If the image is uniform and the arm is not moving, move the arm to the center
      if (uniform_image == true && moving_state_ == false)
          move_arm_center();
  }

private:
  ros::NodeHandle n_;
  std::vector<double> joints_last_position_;
  bool moving_state_;
  ros::ServiceClient client_;
  ros::Subscriber sub1_;
  ros::Subscriber sub2_;
};

int main(int argc, char** argv)
{
    // Initialize the look_away node and create a handle to it
    ros::init(argc, argv, "look_away");
    LookAwayPubSub LookAwayNode;
    // Handle ROS communication events
    ros::spin();
    return 0;
}
