#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>

// States
enum State {
  INIT,
  PICKUP,
  PICKUP_DONE,
  TO_DROPOFF,
  DROPOFF
};

State currentState = INIT;

void pickupCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Received pickup message: %s", msg->data.c_str());

  if (msg->data == "Pickup location reached")
    currentState= PICKUP;
}

// Callback function for the dropoff topic
void dropoffCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Received dropoff message: %s", msg->data.c_str());

  if (msg->data == "Dropoff location reached")
    currentState = DROPOFF;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // ros::Subscriber odom_sub = n.subscribe("odom", 1000, odomCallback);
  ros::Subscriber pickup_sub = n.subscribe("pickup_topic", 1000, pickupCallback);
  ros::Subscriber dropoff_sub = n.subscribe("dropoff_topic", 1000, dropoffCallback);
  
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

   // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    ros::spinOnce();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    switch (currentState) {
      case PICKUP:
      
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = 1.0;
        marker.pose.position.y = 2.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
        ROS_INFO("Pick-up marker displayed");
        ros::Duration(3.0).sleep();
        currentState = PICKUP_DONE;
        break;


      case PICKUP_DONE:

        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        ROS_INFO("Pick-up marker removed");
        currentState = TO_DROPOFF;
        break;

      case TO_DROPOFF:
        ros::spinOnce();
        break;  
        
      case DROPOFF:
      
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = -1.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
        ROS_INFO("Drop-off marker displayed");
        ros::Duration(5.0).sleep();
        return 0;
       
    }
  }
}
