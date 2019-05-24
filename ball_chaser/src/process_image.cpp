#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <algorithm>
using namespace std;

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x=lin_x;
	srv.request.angular_z=ang_z;

	if (!client.call(srv))
        ROS_ERROR("Failed to call service safe_move");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int white_pixel = 255;
	bool moved=false;
	int **greyImage;
	int minGrey=255;
	int maxGrey=0;

	// Create grayscale image matrix
	greyImage = new int*[img.width];
	for(int x = 0; x < img.width; x++){
		greyImage[x] = new int[img.height];
	}

	// Write grayscale image
	for (int y = 0; y < img.height; y++) {
		for (int i = 0; i < img.step; i+=3) {
			int imgDataIndex = i+y*img.step;
			greyImage[i/3][y] = ( img.data[imgDataIndex] + img.data[imgDataIndex+1] + img.data[imgDataIndex+2] )/3;
			minGrey = min(minGrey, greyImage[i/3][y]);
			maxGrey = max(maxGrey, greyImage[i/3][y]);
		}
	}

	// Normalization	
	for (int y = 0; y < img.height; y++) {
		for (int x = 0; x < img.width; x++) {
			greyImage[x][y] = (greyImage[x][y]-minGrey)*255/(maxGrey-minGrey);
			
		}
	}
	

    // Loop through each pixel in the image and check if there's a bright white one
	for (int y = 0; y < img.height; y++) {
		for (int x = 0; x < img.width; x++) {
			maximum = max(maximum, greyImage[x][y]);
		    if (greyImage[x][y]>250) {
		        if(x<img.width/3){
					drive_robot(0, 0.5);
					moved=true;
				}else if(x>img.width-img.width/3){
					drive_robot(0, -0.5);
					moved=true;
				}else{
					drive_robot(0.5, 0);
					moved=true;
				}
		        break;
		    }
		}
	}

	if(moved==false){
		drive_robot(0, 0);
	}

	delete[] greyImage;
	
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

