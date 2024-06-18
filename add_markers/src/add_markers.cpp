#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <complex>


//Positions and thresholds
float pickUp[3] = {1.0, 2.0, 1.0};
float dropOff[3] = {-1.0, 0.0, 1.0};
float thresh[2] = {0.3, 0.1};


//Flags
bool atPickUp = false;
bool atDropOff = false;
bool pickUpDone = false;
bool dropOffDone = false;


void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{ 
  
//Pick up
if (std::abs(pickUp[0] -msg->pose.pose.position.x) < thresh[0] && std::abs(pickUp[1] -msg->pose.pose.position.y) < thresh[0] && std::abs(pickUp[2] -msg->pose.pose.orientation.w) < thresh[1])
   { 
    if(!atPickUp)
    {
     atPickUp = true;
    }
   }else{atPickUp = false;}

//Drop off
if (std::abs(dropOff[0] -msg->pose.pose.position.x) < thresh[0] && std::abs(dropOff[1] -msg->pose.pose.position.y) < thresh[0] && std::abs(dropOff[2] -msg->pose.pose.orientation.w) < thresh[1])
  { 
    if(!atDropOff)
    {
     atDropOff = true;
    }
   }else{atDropOff = false;}

}

int main( int argc, char** argv )
{
  ROS_INFO("Main");
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, chatterCallback);
  


  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
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

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pickUp[0];
    marker.pose.position.y = pickUp[1];
    marker.pose.position.z = 0;
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = pickUp[2];

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Subscriber to the marker not yet recognized.");
      sleep(1);
    }
    
   marker_pub.publish(marker);
   ROS_INFO("Pick-up marker displayed");
   
   //Wait for Pick-Up
   while(!atPickUp)
   {
    ros::spinOnce();
   }
   
   if(atPickUp && !pickUpDone)
   {
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ROS_INFO("Pick-up marker removed");
    pickUpDone = true;
   }  
   
   //Wait for Drop-Off
   while(!atDropOff)
   {
    ros::spinOnce();
   }

   if(atDropOff && !dropOffDone)
   {
    marker.pose.position.x = dropOff[0];
    marker.pose.position.y = dropOff[1];
    marker.pose.orientation.w = dropOff[2];;
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
    ROS_INFO("Drop-off marker displayed");
    dropOffDone = true;
    ros::Duration(5.0).sleep();
   }  
    return 0;
  }
 
}