#include "ros/ros.h"
#include "PPOscilateMotionGenerator.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "PPOscilate_motion_genrator_node");

  ros::NodeHandle nh;
  double frequency = 300.0;


  // Parameters
  std::string input_topic_name;
  std::string output_topic_name;
  std::string output_filtered_topic_name;

  double SwipeVelocity;
  double OrthogonalDamping;
  std::vector<double>  Target_1;
  std::vector<double>  Target_2;
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

  if (!nh.getParam("SwipeVelocity", SwipeVelocity))   {
    ROS_ERROR("Couldn't retrieve the swiping velocity. ");
    // return -1;
  }

  if (!nh.getParam("OrthogonalDamping", OrthogonalDamping))  {
    ROS_ERROR("Couldn't retrieve the orthogonal damping ");
    // return -1;
  }

  if (!nh.getParam("Target_1", Target_1)) {
    ROS_ERROR("Couldn't retrieve the first target. ");
    // return -1;
  }


  if (!nh.getParam("Target_2", Target_2)) {
    ROS_ERROR("Couldn't retrieve the second target. ");
    // return -1;
  }

  if (bPublish_DS_path)
        ROS_INFO("Starting the PPoscilate Motion generator... Publishing path in this node. ");
  else
      ROS_INFO("Starting the PPoscilate Motion generator... NOT Publishing path in this node. ");


  PPOscilateMotionGenerator PPOscilate_motion_generator(nh,
      frequency,
      input_topic_name,
      output_topic_name,
      output_filtered_topic_name,
      SwipeVelocity,
      OrthogonalDamping,
      Target_1,
      Target_2,
      bPublish_DS_path);

  if (!PPOscilate_motion_generator.Init()) {
    return -1;
  }
  else {
    PPOscilate_motion_generator.Run();
  }


  return 0;
}
