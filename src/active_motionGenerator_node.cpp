#include "ros/ros.h"
#include "DSMotionGenerator.h"





int main(int argc, char **argv)
{
  ros::init(argc, argv, "active_motion_genrator_node");

  ros::NodeHandle nh;
  double frequency = 300.0;


  // Parameters
  std::string path_to_cds_gmm;
  double max_des_vel;


  if (!nh.getParam("gmm_path", path_to_cds_gmm))
  {
    ROS_ERROR("Couldn't retrieve the gmm path. ");
    // return -1;
  }


  if (!nh.getParam("max_des_vel", max_des_vel))
  {
    ROS_ERROR("Couldn't retrieve the maximum desired velocity max_des_vel. ");
    // return -1;
  }

  ROS_INFO("Starting the Motion generator...");

  DSMotionGenerator ds_motion_generator(nh, frequency,path_to_cds_gmm, max_des_vel);
  ds_motion_generator.Init();

  return 0;
}
