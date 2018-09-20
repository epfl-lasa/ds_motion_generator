#ifndef __PPOSCILATE_MOTION_GENERATOR_H__
#define __PPOSCILATE_MOTION_GENERATOR_H__

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"


#include <vector>

#include "MathLib.h"
#include "CDDynamics.h"

#include <mutex>


#include <dynamic_reconfigure/server.h>
#include <ds_motion_generator/PPOscilate_paramsConfig.h>

class PPOscilateMotionGenerator {


private:

	double SwipeVelocity_;
	int SwipeDirection_;
	double OrthogonalDamping_;

	std::vector<double> Target_1_;
	std::vector<double> Target_2_;

	double vel_limit_;

	double SwipeVel_offset_;
	double Orth_damp_scaling_;
    bool bPublish_DS_path_;

	int TARGET_id;

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

    geometry_msgs::Pose  msg_real_pose_;
    geometry_msgs::Twist msg_desired_velocity_;
    geometry_msgs::Twist msg_desired_velocity_filtered_;

	nav_msgs::Path msg_DesiredPath_;
	int MAX_FRAME = 200;



	//dynamic reconfig settig
	dynamic_reconfigure::Server<ds_motion_generator::PPOscilate_paramsConfig> dyn_rec_srv_;
	dynamic_reconfigure::Server<ds_motion_generator::PPOscilate_paramsConfig>::CallbackType dyn_rec_f_;

	// Class variables
	std::mutex mutex_;

	MathLib::Vector real_pose_;
	MathLib::Vector target_pose_;	
	MathLib::Vector other_target_pose_;


	MathLib::Vector target_1_offset_;
	MathLib::Vector target_2_offset_;
	MathLib::Vector PP_line_;



	MathLib::Vector desired_velocity_;
	MathLib::Vector desired_velocity_filtered_;

	double scaling_factor_;



public:
	PPOscilateMotionGenerator(ros::NodeHandle &n,
	                     double frequency,
	                     std::string input_topic_name,
	                     std::string output_topic_name,
	                     std::string output_filtered_topic_name,
	                     double SwipeVelocity,
	                     double OrthogonalDamping,
	                     std::vector<double> Target_1,
                         std::vector<double> Target_2,
                         bool bPublish_DS_path);

	bool Init();

	void Run();

private:

	bool InitializeDS();

	bool InitializeROS();

	void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

	void ComputeDesiredVelocity();

	void PublishDesiredVelocity();

	void PublishFuturePath();


	void DynCallback(ds_motion_generator::PPOscilate_paramsConfig &config, uint32_t level);

};


#endif
