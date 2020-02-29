/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author:  Mahdi Khoramshahi
 * email:   mahdi.khoramshahi@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the European Communitys Horizon 2020 Research and Innovation 
 * programme ICT-23-2014, grant agreement 643950-SecondHands.
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

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
  std::string world_frame_name;

  double SwipeVelocity;
  double OrthogonalDamping;
  std::vector<double>  Target_1;
  std::vector<double>  Target_2;
  bool bPublish_DS_path;




  if (!nh.getParam("input_topic_name", input_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    return -1;
  }

  if (!nh.getParam("output_topic_name", output_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the output. ");
    return -1;
  }

  if (!nh.getParam("world_frame_name", world_frame_name))   {
    ROS_ERROR("Couldn't retrieve the world reference frame name for the output. ");
    return -1;
  }



  if (!nh.getParam("output_filtered_topic_name", output_filtered_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the filtered output. ");
    return -1;
  }

  if (!nh.getParam("publish_DS_path", bPublish_DS_path))   {
    ROS_ERROR("Couldn't retrieve the publish DS path boolean. ");
    return -1;
  }

  if (!nh.getParam("SwipeVelocity", SwipeVelocity))   {
    ROS_ERROR("Couldn't retrieve the swiping velocity. ");
    return -1;
  }

  if (!nh.getParam("OrthogonalDamping", OrthogonalDamping))  {
    ROS_ERROR("Couldn't retrieve the orthogonal damping ");
    return -1;
  }

  if (!nh.getParam("Target_1", Target_1)) {
    ROS_ERROR("Couldn't retrieve the first target. ");
    return -1;
  }


  if (!nh.getParam("Target_2", Target_2)) {
    ROS_ERROR("Couldn't retrieve the second target. ");
    return -1;
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
      bPublish_DS_path,
      world_frame_name);

  if (!PPOscilate_motion_generator.Init()) {
    return -1;
  }
  else {
    PPOscilate_motion_generator.Run();
  }


  return 0;
}
