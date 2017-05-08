#ifndef __DS_MOTION_GENERATOR_H__
#define __DS_MOTION_GENERATOR_H__

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include <vector>

#include "MathLib.h"
#include "GMRDynamics.h"
#include "CDDynamics.h"

#include <mutex>


class DSMotionGenerator {


private:

	double max_desired_vel_;

	std::string input_topic_name_;
	std::string output_topic_name_;
	std::string output_filtered_topic_name_;

	int K_gmm_;
	int dim_;
	std::vector<double> Priors_;
	std::vector<double> Mu_;
	std::vector<double> Sigma_;
	double dt_;

	double Wn_;
	std::vector<double> filter_VelLimits_;
	std::vector<double> filter_AccLimits_;


	std::mutex mutex_;


	std::unique_ptr<GMRDynamics> SED_GMM_;
	std::unique_ptr<CDDynamics> CDDyn_filter_;



	// A handle to the node in ros
	ros::NodeHandle nh_;
	// Rate of the run loop
	ros::Rate loop_rate_;

	ros::Subscriber sub_real_pose_;
	ros::Publisher pub_desired_twist_;
	ros::Publisher pub_desired_twist_filtered_;

	MathLib::Vector real_pose_;
	MathLib::Vector desired_velocity_;
	MathLib::Vector desired_velocity_filtered_;

	geometry_msgs::Pose msg_real_pose_;
	geometry_msgs::Twist msg_desired_velocity_;
	geometry_msgs::Twist msg_desired_velocity_filtered_;


public:
	DSMotionGenerator(ros::NodeHandle &n,
	                  double frequency,
	                  int K_gmm,
	                  int dim,
	                  std::vector<double> Priors,
	                  std::vector<double> Mu,
	                  std::vector<double> Sigma,
	                  std::string input_topic_name,
	                  std::string output_topic_name,
	                  std::string output_filtered_topic_name,
	                  double Wn,
                      std::vector<double> VelLimits,
                      std::vector<double> AccLimits,
	                  double max_desired_vel);

	bool Init();

	void Run();

private:

	bool InitializeDS();

	bool InitializeROS();

	void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

	void ComputeDesiredVelocity();

	void PublishDesiredVelocity();


};


#endif
