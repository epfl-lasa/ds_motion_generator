#include "ros/ros.h"
#include "targetTracker.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_tracker_node");

  ros::NodeHandle nh;
  double frequency = 300.0;


  // Parameters
  std::string base_topic_name;
  std::string object_1_topic_name;
  std::string object_2_topic_name;
  std::string target_1_topic_name;
  std::string target_2_topic_name;
  std::string topic_target1_vel;
  std::string topic_target2_vel;

  if (!nh.getParam("base_topic_name", base_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the base. ");
    // return -1;
  }

  if (!nh.getParam("object_1_topic_name", object_1_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the first object. ");
    // return -1;
  }

  if (!nh.getParam("object_2_topic_name", object_2_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the second object. ");
    // return -1;
  }

  if (!nh.getParam("target_1_topic_name", target_1_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the first target. ");
    // return -1;
  }

  if (!nh.getParam("target_2_topic_name", target_2_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the second target. ");
    // return -1;
  }

  if (!nh.getParam("topic_target1_vel", topic_target1_vel))   {
    ROS_ERROR("Couldn't retrieve the topic name for the first target velocity. ");
    // return -1;
  }

  if (!nh.getParam("topic_target2_vel", topic_target2_vel))   {
    ROS_ERROR("Couldn't retrieve the topic name for the second target velocity. ");
    // return -1;
  }




  ROS_INFO("Starting the Motion generator...");

  TargetTracker myTargetTracker(nh, frequency,
                                base_topic_name,
                                object_1_topic_name,
                                object_2_topic_name,
                                target_1_topic_name,
                                target_2_topic_name,
                                topic_target1_vel,
                                topic_target2_vel);

  myTargetTracker.Run();


  return 0;
}
