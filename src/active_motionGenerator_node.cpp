#include "ros/ros.h"
#include "DSMotionGenerator.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "active_motion_genrator_node");

  ros::NodeHandle nh;
  double frequency = 300.0;


  // Parameters
  std::string path_to_cds_gmm;
  double max_des_vel;
  int K_gmm;
  int dim;
  std::vector<double> Priors;
  std::vector<double> Mu;
  std::vector<double> Sigma;


  if (!nh.getParam("gmm_path", path_to_cds_gmm))   {
    ROS_ERROR("Couldn't retrieve the gmm path. ");
    // return -1;
  }

  if (!nh.getParam("K", K_gmm))   {
    ROS_ERROR("Couldn't retrieve the number of guassians. ");
    // return -1;
  }

  if (!nh.getParam("dim", dim))  {
    ROS_ERROR("Couldn't retrieve the dimension of the state space. ");
    // return -1;
  }


  if (!nh.getParam("Priors", Priors))   {
    ROS_ERROR("Couldn't retrieve Priors. ");
    // return -1;
  }

  // std::cout << Priors.size() << std::endl;
  if (!nh.getParam("Mu", Mu))   {
    ROS_ERROR("Couldn't retrieve Mu. ");
    // return -1;
  }

  if (!nh.getParam("Sigma", Sigma))  {
    ROS_ERROR("Couldn't retrieve Sigma. ");
    // return -1;
  }


  if (!nh.getParam("max_des_vel", max_des_vel))  {
    ROS_ERROR("Couldn't retrieve the maximum desired velocity max_des_vel. ");
    // return -1;
  }

  ROS_INFO("Starting the Motion generator...");

  DSMotionGenerator ds_motion_generator(nh, frequency,
                                        K_gmm, dim, Priors, Mu, Sigma,
                                        max_des_vel);
  if (!ds_motion_generator.Init()) {
    return -1;
  }
  else{
    ds_motion_generator.Run();
  }


  return 0;
}
