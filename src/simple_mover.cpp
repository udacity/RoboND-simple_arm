#include "ros/ros.h"
#include "std_msgs/Float64.h"

class SimpleArmPublisher {
public:
  SimpleArmPublisher():
    loop_rate_(10) { // Set loop frequency of 10Hz
      joint1_pub_ = n_.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
      joint2_pub_ = n_.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);
  }

  void Run() {
    // Get ROS start time
    while (not start_time_) {
      start_time_ = ros::Time::now().toSec();
    }

    while (ros::ok()) {
      // Get ROS elapsed Time
      elapsed_ = ros::Time::now().toSec() - start_time_;

      // Set the arm joint angles
      std_msgs::Float64 joint1_angle, joint2_angle;
      joint1_angle.data = sin(2 * M_PI * 0.1 * elapsed_) * (M_PI / 2);
      joint2_angle.data = sin(2 * M_PI * 0.1 * elapsed_) * (M_PI / 2);

      // Publish the arm joint angles
      joint1_pub_.publish(joint1_angle);
      joint2_pub_.publish(joint2_angle);

      // Sleep for the time remaining until 10 Hz is reached
      loop_rate_.sleep();
    }
  }
private:
  // Initialize the arm_mover node
  ros::NodeHandle n_;
  // Create two publishers for the /simple_arm/joint_x_position_controller/command topics
  ros::Publisher joint1_pub_;
  ros::Publisher joint2_pub_;
  int start_time_;
  int elapsed_;
  ros::Rate loop_rate_;
};

int main(int argc, char** argv)
{
  // Initialize the arm_mover node
  ros::init(argc, argv, "arm_mover");
  SimpleArmPublisher SimpleMoverPub;
  SimpleMoverPub.Run();
  return 0;
}
