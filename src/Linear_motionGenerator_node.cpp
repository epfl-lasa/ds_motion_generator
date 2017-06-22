#include "ros/ros.h"
#include "LinearMotionGenerator.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "linear_motion_genrator_node");

  ros::NodeHandle nh;
  double frequency = 300.0;


  // Parameters
  std::string input_topic_name;
  std::string output_topic_name;
  std::string output_filtered_topic_name;

  std::string direction;
  double velocity;





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

  if (!nh.getParam("direction", direction))   {
    ROS_ERROR("Couldn't retrieve the swiping velocity. ");
    // return -1;
  }

  if (!nh.getParam("velocity", velocity))   {
    ROS_ERROR("Couldn't retrieve the velocity. ");
    // return -1;
  }



  ROS_INFO("Starting the Motion generator...");


  LinearMotionGenerator linear_motion_generator(nh,
      frequency,
      input_topic_name,
      output_topic_name,
      output_filtered_topic_name,
      direction,
      velocity);

  if (!linear_motion_generator.Init()) {
    return -1;
  }
  else {
    linear_motion_generator.Run();
  }


  return 0;
}