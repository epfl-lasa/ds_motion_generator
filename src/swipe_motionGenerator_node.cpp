#include "ros/ros.h"
#include "SwipeMotionGenerator.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "swipe_motion_genrator_node");

  ros::NodeHandle nh;
  double frequency = 300.0;


  // Parameters
  std::string input_topic_name;
  std::string output_topic_name;
  std::string output_filtered_topic_name;

  int SwipeDirection;
  double SwipeVelocity;
  double OrthogonalDamping;
  double OrthVelLim;
  std::vector<double>  SwipeTarget;
  bool bPublish_DS_path;



  if (!nh.getParam("input_topic_name", input_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("output_topic_name", output_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the output. ");
    // return -1;
  }

  if (!nh.getParam("output_filtered_topic_name", output_filtered_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the filtered output. ");
    // return -1;
  }

  if (!nh.getParam("publish_DS_path", bPublish_DS_path))   {
    ROS_ERROR("Couldn't retrieve the publish DS path boolean. ");
    // return -1;
  }

  if (!nh.getParam("SwipeDirection", SwipeDirection))   {
    ROS_ERROR("Couldn't retrieve the swiping velocity. ");
    // return -1;
  }

  if (!nh.getParam("SwipeVelocity", SwipeVelocity))   {
    ROS_ERROR("Couldn't retrieve the swiping velocity. ");
    // return -1;
  }

  if (!nh.getParam("OrthogonalDamping", OrthogonalDamping))  {
    ROS_ERROR("Couldn't retrieve the orthogonal damping ");
    // return -1;
  }

  if (!nh.getParam("SwipeTarget", SwipeTarget)) {
  ROS_ERROR("Couldn't retrieve the swiping target. ");
    // return -1;
  }

  if (bPublish_DS_path)
        ROS_INFO("Starting the Swipe Motion generator... Publishing path in this node. ");
  else
      ROS_INFO("Starting the Swipe Motion generator... NOT Publishing path in this node. ");

  SwipeMotionGenerator swipe_motion_generator(nh,
      frequency,
      input_topic_name,
      output_topic_name,
      output_filtered_topic_name,
      SwipeDirection,
      SwipeVelocity,
      OrthogonalDamping,
      SwipeTarget,
      bPublish_DS_path);

  if (!swipe_motion_generator.Init()) {
    return -1;
  }
  else {
    swipe_motion_generator.Run();
  }


  return 0;
}
