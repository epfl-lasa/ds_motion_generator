#ifndef __TARGET_TRACKER_H__
#define __TARGET_TRACKER_H__

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include <vector>

class TargetTracker {


private:

	// ROS variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;

	ros::Subscriber sub_base_pose_;
	ros::Subscriber sub_object1_pose_;
	ros::Subscriber sub_object2_pose_;

	ros::Publisher pub_target1_pose_;
	ros::Publisher pub_target2_pose_;

	std::vector<double> base_position_;


public:
	TargetTracker(ros::NodeHandle &n,
	              double frequency,
	              std::string base_topic_name,
	              std::string object1_topic_name,
	              std::string object2_topic_name,
	              std::string target1_topic_name,
	              std::string target2_topic_name);

	void Run();

private:


	void UpdateBasePosition(const geometry_msgs::PoseStamped::ConstPtr& msg);

	void UpdateTarget_1_Position(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void UpdateTarget_2_Position(const geometry_msgs::PoseStamped::ConstPtr& msg);

};


#endif
