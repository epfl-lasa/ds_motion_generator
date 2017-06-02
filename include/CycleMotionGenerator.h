#ifndef __CYCLE_MOTION_GENERATOR_H__
#define __CYCLE_MOTION_GENERATOR_H__

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
// #include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"


#include <vector>

#include "MathLib.h"
#include "CDDynamics.h"

#include <mutex>


#include <dynamic_reconfigure/server.h>
#include <ds_motion_generator/CYCLE_paramsConfig.h>

class CycleMotionGenerator {


private:

	std::vector<double> Cycle_Target_;

	double Cycle_radius_;
	double Cycle_radius_scale_;

	double Cycle_speed_;
	double Cycle_speed_offset_;

	double Convergence_Rate_;
	double Convergence_Rate_scale_;

	double Velocity_limit_;

	// Filter variables
	std::unique_ptr<CDDynamics> CCDyn_filter_;

	double dt_;
	double Wn_;
	MathLib::Vector accLimits_;
	MathLib::Vector velLimits_;


	// ROS variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;

	ros::Subscriber sub_real_pose_;
	ros::Publisher pub_desired_twist_;
	ros::Publisher pub_desired_twist_filtered_;
	ros::Publisher pub_target_;
	ros::Publisher pub_DesiredPath_;

	std::string input_topic_name_;
	std::string output_topic_name_;
	std::string output_filtered_topic_name_;

	geometry_msgs::Pose msg_real_pose_;
	geometry_msgs::TwistStamped msg_desired_velocity_;
	geometry_msgs::TwistStamped msg_desired_velocity_filtered_;

	nav_msgs::Path msg_DesiredPath_;
	int MAX_FRAME = 200;



	//dynamic reconfig settig
	dynamic_reconfigure::Server<ds_motion_generator::CYCLE_paramsConfig> dyn_rec_srv_;
	dynamic_reconfigure::Server<ds_motion_generator::CYCLE_paramsConfig>::CallbackType dyn_rec_f_;


	// Class variables
	std::mutex mutex_;

	MathLib::Vector real_pose_;
	MathLib::Vector target_pose_;
	MathLib::Vector target_offset_;

	MathLib::Vector desired_velocity_;
	MathLib::Vector desired_velocity_filtered_;




public:
	CycleMotionGenerator(ros::NodeHandle &n,
	                     double frequency,
	                     std::string input_topic_name,
	                     std::string output_topic_name,
	                     std::string output_filtered_topic_name,
	                     std::vector<double> CenterRotation,
	                     double radius,
	                     double RotationSpeed,
	                     double ConvergenceRate);

	bool Init();

	void Run();

private:

	bool InitializeDS();

	bool InitializeROS();

	void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

	void ComputeDesiredVelocity();

	void PublishDesiredVelocity();

	void PublishFuturePath();


	void DynCallback(ds_motion_generator::CYCLE_paramsConfig &config, uint32_t level);

};


#endif
