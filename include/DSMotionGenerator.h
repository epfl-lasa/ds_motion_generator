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


#include <dynamic_reconfigure/server.h>
#include <example_ds/SED_paramsConfig.h>

class DSMotionGenerator {


private:

	// DS variables
	std::unique_ptr<GMRDynamics> SED_GMM_;

	int K_gmm_;
	int dim_;
	std::vector<double> Priors_;
	std::vector<double> Mu_;
	std::vector<double> Sigma_;
	double dt_;

	double max_desired_vel_;

	// Filter variables
	std::unique_ptr<CDDynamics> CCDyn_filter_;

	double Wn_;
	MathLib::Vector accLimits_;
	MathLib::Vector velLimits_;


	// ROS variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;

	ros::Subscriber sub_real_pose_;
	ros::Publisher pub_desired_twist_;
	ros::Publisher pub_desired_twist_filtered_;

	std::string input_topic_name_;
	std::string output_topic_name_;
	std::string output_filtered_topic_name_;

	geometry_msgs::Pose msg_real_pose_;
	geometry_msgs::Twist msg_desired_velocity_;
	geometry_msgs::Twist msg_desired_velocity_filtered_;



	//dynamic reconfig settig
	dynamic_reconfigure::Server<example_ds::SED_paramsConfig> dyn_rec_srv_;
	dynamic_reconfigure::Server<example_ds::SED_paramsConfig>::CallbackType dyn_rec_f_;


	// Class variables
	std::mutex mutex_;

	MathLib::Vector real_pose_;
	MathLib::Vector target_pose_;

	MathLib::Vector desired_velocity_;
	MathLib::Vector desired_velocity_filtered_;

	double scaling_factor_;
	double ds_vel_limit_;



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
	                  std::string output_filtered_topic_name);

	bool Init();

	void Run();

private:

	bool InitializeDS();

	bool InitializeROS();

	void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

	void ComputeDesiredVelocity();

	void PublishDesiredVelocity();

	void DynCallback(example_ds::SED_paramsConfig &config, uint32_t level);

};


#endif
