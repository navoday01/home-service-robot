#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Establish this program as a ROS node.
  ros::NodeHandle nh;

  // Create a publisher object.
  ros::Publisher pickup_pub = nh.advertise<std_msgs::String>("pickup_topic", 1000);
  ros::Publisher dropoff_pub = nh.advertise<std_msgs::String>("dropoff_topic", 1000);

  std_msgs::String pickup_msg;
  std_msgs::String dropoff_msg;


  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pick;

  // set up the frame parameters
  pick.target_pose.header.frame_id = "map";
  pick.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  pick.target_pose.pose.position.x = 1.0;
  pick.target_pose.pose.position.y = 2.0;
  pick.target_pose.pose.orientation.w = 1.0;

   // Send the pick position and orientation for the robot to reach
  ROS_INFO("Sending pick position");
  ac.sendGoal(pick);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot reached PICK-UP location.");

    // Create and fill in the pickup message.
    pickup_msg.data = "Pickup location reached";

    // Publish the message.
    pickup_pub.publish(pickup_msg);
    ROS_INFO("Published: %s", pickup_msg.data.c_str());
    ros::spinOnce();

    ros::Duration(3.0).sleep();

    move_base_msgs::MoveBaseGoal drop;

    // set up the frame parameters
    drop.target_pose.header.frame_id = "map";
    drop.target_pose.header.stamp = ros::Time::now();
    
    // Define a position and orientation for the robot to reach
    drop.target_pose.pose.position.x = -1.0;
    drop.target_pose.pose.position.y = 0.0;
    drop.target_pose.pose.orientation.w = 1.0;

    // Send the drop position and orientation for the robot to reach
    ROS_INFO("Sending drop position");
    ac.sendGoal(drop);
    
    // Wait an infinite time for the results
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Robot reached DROP-OFF location.");

      // Create and fill in the dropoff message.
      dropoff_msg.data = "Dropoff location reached";

      // Publish the message.
      dropoff_pub.publish(dropoff_msg);
      ROS_INFO("Published: %s", dropoff_msg.data.c_str());
      ros::spinOnce();

      ros::Duration(3.0).sleep();
    }
    else 
    {
      ROS_INFO("Robot failed to reach DROP-OFF location.");
    }

    }
  else
  {
    ROS_INFO("Robot failed to reach PICK-UP location.");
  }
    

  return 0;
}
