#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// Global joint publisher objects
ros::Publisher motor_command_publisher;



// This callback function executes whenever a command_robot service is requested
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{

    ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);

	// 1 publish requested linear x and angular velocities
    geometry_msgs::Twist motor_command;
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
	// publish it!
    motor_command_publisher.publish(motor_command);

	// 2 return requested velocities
    res.msg_feedback = "Vel set to: linear_x:" + std::to_string((double)motor_command.linear.x) + " angular_z: " + std::to_string((double)motor_command.angular.z);
	// response
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node and create a handle to it
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    //Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define two publishers to publish std_msgs::Float64 messages on joints respective topics
    //joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    //joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    // Define a safe_move service with a handle_safe_move_request callback function
    //ros::ServiceServer service = n.advertiseService("/arm_mover/safe_move", handle_safe_move_request);
    //ROS_INFO("Ready to send joint commands");

    //while (ros::ok()) {
	// Create a motor_command object
	//geometry_msgs::Twist motor_command; DONE
	// Set wheel velocities
	//motor_command.linear.x = 0.5; DONE (req.linear_x)
	//motor_command.angular.z = 0.0; DONE
	// Published angles to drive the robot
	//motor_command_publisher.publish(motor_command);DONE
    //}



    // Handle ROS communication events
    ros::spin();

    return 0;
}
