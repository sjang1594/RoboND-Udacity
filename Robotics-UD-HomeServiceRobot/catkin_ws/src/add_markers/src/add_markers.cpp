#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
using namespace std;

//The basic code was copied from ROS.org : http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

// The pick up location position is 2.0(x), 6.0(y), 1.0(w).
// The deliver location position is -2.0(x), 3.0(y), 1.0(w).
float pickUpLoc[3] = {2.0, 3.0, 1.0};
float deliverLoc[3] = {-2.0, 4.0, 1.0};

//Define flag whether the object was picked up or delivered. 
bool isPickedUp = false;
bool isDelivered = false;

//Define the call_back function for odometry
void odoom_call_back(const nav_msgs::Odometry::ConstPtr &msg){
  ROS_INFO("Received Message!");
  float x_pos;
  float y_pos;
  
  x_pos = msg->pose.pose.position.x;
  y_pos = msg->pose.pose.position.y;

  //Declare & initialize variables for distance for pickUpLocation and deliver location
  float x_distance = 0.0;
  float y_distance = 0.0;
  float x1_distance = 0.0;
  float y1_distance = 0.0;

  //Check the distance between odometry and pickUp Location.
  x_distance = abs(-pickUpLoc[0] - x_pos);
  y_distance = abs(-pickUpLoc[1] - y_pos);
  
  //Check the distance between odometry and deliverd Location.
  x1_distance = abs(-deliverLoc[0] - x_pos);
  y1_distance = abs(-deliverLoc[1] - y_pos);
 
  //Compare the distance with threshold for pickUp Location.
  if(x_distance < 1 && y_distance < 1){
    isPickedUp = true;
  }
  //Compare the distance with threshold for deliver Location.
  if(isPickedUp == true && x1_distance < 1 && y1_distance < 1){
    isDelivered = true;
  }
  ROS_INFO("X_POSE : %f, Y_POS : %f", x_pos, y_pos);
}

//Main Function.
int main( int argc, char** argv ){
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1); 
  ros::Subscriber odometry_sub = n.subscribe("/odom", 1, odoom_call_back);
  ros::Rate loop_rate(0.1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  
  while (ros::ok()){
    visualization_msgs::Marker marker;
  
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
 
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;
   
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
    
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
 
    //Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header 
    marker.pose.position.x = pickUpLoc[0];
    marker.pose.position.y = pickUpLoc[1];
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.w = pickUpLoc[2];

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
 
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1){
      if (!ros::ok()){
           return 0;
      }
           ROS_WARN_ONCE("Please create a subscriber to the marker");
           sleep(1);
    }
     //Create pick-location marker.
     marker_pub.publish(marker);
     
     //Loop Until the robot is  pick-up location. 
     while(!isPickedUp){
       //call single-threaded spinning. -> Within the while loop spinOnce() meaning that use a call back(an authority) 
     //to user for messaging into queue.
       ros::spinOnce();
     }
     if(isPickedUp == true){
     //Delete a marker for current marker
     marker.action = visualization_msgs::Marker::DELETE;
     marker_pub.publish(marker);
     ROS_INFO("Arrived in pick up location");
    
     //After 5 sec...
     ros::Duration(5.0).sleep();
     
     //Setting up New location for delivery.
     marker.pose.position.x = deliverLoc[0];
     marker.pose.position.y = deliverLoc[1];
     marker.pose.position.z = 0;
     
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = deliverLoc[3];
     
     marker.action = visualization_msgs::Marker::ADD;
     shape  = visualization_msgs::Marker::SPHERE;
     marker_pub.publish(marker);
     }
    
   while(!isDelivered){
     //Within the while loop spinOnce() meaning that use a call back(an authority) 
     //to user for messaging into queue.
     ros::spinOnce();
   }
   if(isDelivered == true){
     marker.action = visualization_msgs::Marker::DELETE;
     marker_pub.publish(marker);
     ROS_INFO("Delivered");
   }
   
   //After 5 sec.. 
   ros::Duration(5.0).sleep();
   loop_rate.sleep();

   return 0;
}
}


