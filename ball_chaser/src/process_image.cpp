#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Moving the bot in the specified direction");

    // Request centered joint angles [1.57, 1.57]
    //simple_arm::GoToPosition srv;
    //srv.request.joint_1 = 1.57;
    //srv.request.joint_2 = 1.57;

    // Call the safe_move service and pass the requested joint angles
    //if (!client.call(srv))
    //    ROS_ERROR("Failed to call service safe_move");

    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget drive_command;
    drive_command.request.linear_x = lin_x;
    drive_command.request.angular_z = ang_z;

    // Call the DriveToTarget service
    if (!client.call(drive_command))
        ROS_ERROR("Failed to call service DriveToTarget");

}



// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    //bool uniform_image = true;

    // Loop through each pixel in the image and check if its equal to the first one
    //for (int i = 0; i < img.height * img.step; i++) {
    //    if (img.data[i] - img.data[0] != 0) {
    //        uniform_image = false;
    //        break;
    //    }
    //}

    // If the image is uniform and the arm is not moving, move the arm to the center
    //if (uniform_image == true && moving_state == false)
    //    move_arm_center();

    int white_pixel = 255;
    bool white_flag = false;
    float x, z;
    x = 0;


    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.height; i++) {
	for (int j=0; j < img.step; ++j) {

            if (img.data[i * img.step + j] == white_pixel) {
	        white_flag = true;
                if (j <= 1100) {
	    	    z = .1;
		    //break;
                }
	        else if(j > 1100 && j < 1300) {
		    z = 0;
		    //break;
		    //add linear_x drive command, here
	        }
	        else if(j >= 1300) {
		    z = -.1;
		    //break;
	        }
            }
	}
	
    }
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    if (white_flag==false){
	x = 0;
	z = 0;
    }

    drive_robot(x, z);


}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to rgb_camera/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
