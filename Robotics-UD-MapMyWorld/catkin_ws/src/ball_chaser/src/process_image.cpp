#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float x, float z)
{   
    ROS_INFO_STREAM("Robot is driving for the white ball!");
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = x;
    srv.request.angular_z = z;
    // TODO: Request a service and pass the velocities to it to drive the robot
    if (!client.call(srv))
        ROS_ERROR("Failed to call service driveToTarget");
}

// This callback function executes and reads the image data continuously 
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    //Define the height of the image, up/down, and step for left/right.
    int img_height = img.height;
    int img_step = img.step;
    
    //Count for detection.    
    int cnt_for_detection = 0;

    //Error from center step
    float off_set = 0.0;

    //Accumulate the error
    float accm_offset = 0.0;
    //Input linear velocity in x axis 
    float Vel_X = 0.0;
    //Input Angular velocity in z axis  
    float Vel_Z = 0.0;

    for (int i = 0; i < img_height ; i++) {
        for (int j = 1; j < img_step-1; j++) {
            // img.data has RGB data.
            // If white pixel was found in the image sequentially/consecutively.
            if (img.data[i * img_step + j-1] == white_pixel && img.data[i * img_step + j] == white_pixel && img.data[i * img_step + j+1] == white_pixel) { 

                //Better approach is to find the offset from the center of image, then add all the 
                //offsets and calculate how many white pixels were detected. 
                off_set = j - img_step/2;
		accm_offset += off_set;
                cnt_for_detection++;
            }
        }
    }
    
    //If there were white pixels(white ball) were found in the image, command to robot with certain velocity in x and z axis.
    if (cnt_for_detection > 0){
       Vel_X = 0.5;
       
       //Since Velocity in Z axis is responsible for rotation of robot towards to white ball, find the mean of offset and Normalize from -1 to 1.
       float mean_offset = accm_offset / (img_step / 2.0);
       float norm_offset = mean_offset / cnt_for_detection;
       
       Vel_Z = -4.0 * norm_offset;
       ROS_INFO_STREAM("Angular Velocity: "+ std::to_string(Vel_Z));

       //If the valocity of Z - axis is faster than 1.2, then threshold to 0.5 vice versa.
       if(Vel_Z > 1.2){
          Vel_Z = 0.5;
       }
       if(Vel_Z < -1.2){
          Vel_Z = -0.5;
       }
    }
    
    //When the white ball does not found in the image, stop the robot.
    else{Vel_X = 0.0; Vel_Z = 0.0;}
    
    drive_robot(Vel_X, Vel_Z);

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 5, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
