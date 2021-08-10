#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char** argv){

	// Initialise the arm_mover node
	ros::init(argc, argv, "arm_mover");
	
	// Create a handle to the arm_mover node
	ros::NodeHandle n;

	// Create a publisher that can publish a std_msgs::Float64 message on the /simple_arm/joint_1_position_controller/command topic
	ros::Publisher joint1Pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
	// Create a publisher that can publish a std_msgs::Float64 message on the /simple_arm/joint_2_position_controller/command topic
	ros::Publisher joint2Pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

	// Set loop frequency of 10Hz
	ros::Rate loop_rate(10);

	int startTime, elapsed;
	
	// Get ROS start time
	while(not startTime){
		startTime = ros::Time::now().toSec();	
	} 

	while(ros::ok()){
		// Get ROS elapsed time
		elapsed = ros::Time::now().toSec() - startTime;

		// Set the am joint angles
		std_msgs::Float64 joint1Angle, joint2Angle; 
		joint1Angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);
		joint2Angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);

		// Publish the arm join angles
		joint1Pub.publish(joint1Angle); 
		joint2Pub.publish(joint2Angle);

		// Sleep for the time remaining until 10Hz is reached	
		loop_rate.sleep();	
	}
	
	return 0;
	
}
