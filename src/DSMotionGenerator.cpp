#include "DSMotionGenerator.h"
#include <tf/transform_datatypes.h>




DSMotionGenerator::DSMotionGenerator(ros::NodeHandle &n,
                                     double frequency,
                                     int K_gmm,
                                     int dim,
                                     std::vector<double> Priors,
                                     std::vector<double> Mu,
                                     std::vector<double> Sigma,
                                     double max_desired_vel)
	: nh_(n),
	  loop_rate_(frequency),
	  K_gmm_(K_gmm),
	  dim_(dim),
	  Priors_(Priors),
	  Mu_(Mu),
	  Sigma_(Sigma),
	  max_desired_vel_(max_desired_vel),
	  dt_(1 / frequency) {

	ROS_INFO_STREAM("Motion generator node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


bool DSMotionGenerator::Init() {

	real_pose_.Resize(6);
	desired_velocity_.Resize(6);

	if (!InitializeDS()) {
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	if (!InitializeROS()) {
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	return true;
}


bool DSMotionGenerator::InitializeDS() {

	if (Priors_.size() != K_gmm_) {
		ROS_ERROR_STREAM("InitializeDS: " << K_gmm_ << " priors is expected while " << Priors_.size() << " is provided.");
		return false;
	}

	if (Mu_.size() != K_gmm_ * dim_) {
		ROS_ERROR_STREAM("InitializeDS: " << K_gmm_ * dim_ << " elements in Mu is expected while " << Mu_.size() << " is provided.");
		return false;
	}

	if (Sigma_.size() != K_gmm_ * dim_ * dim_ ) {
		ROS_ERROR_STREAM("InitializeDS: " << K_gmm_ * dim_ * dim_ << " elements in Sigma is expected while " << Sigma_.size() << " is provided.");
		return false;
	}

	SED_GMM_.reset (new GMRDynamics(K_gmm_, dim_, dt_, Priors_, Mu_, Sigma_ ));
	SED_GMM_->initGMR(0, 2, 3, 5 );
	return true;

}


bool DSMotionGenerator::InitializeROS() {

	pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>("/ds/desired_velocity", 1);
	sub_real_pose_ = nh_.subscribe("/ds/real_position" , 1000,
	                               &DSMotionGenerator::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());

	if (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The Motion generator is ready.");
		return true;
	}
	else {
		ROS_ERROR("The ros node has a problem.");
		return false;
	}

}


void DSMotionGenerator::Run() {

	while (nh_.ok()) {

		ComputeDesiredVelocity();
		
		PublishDesiredVelocity();

		ROS_INFO_STREAM_THROTTLE(1, "Spinning!");

		ros::spinOnce();

		loop_rate_.sleep();
	}
}

void DSMotionGenerator::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	msg_real_pose_ = *msg;

	real_pose_(0) = msg_real_pose_.position.x;
	real_pose_(1) = msg_real_pose_.position.y;
	real_pose_(2) = msg_real_pose_.position.z;

	double qtx = msg_real_pose_.orientation.x;
	double qty = msg_real_pose_.orientation.y;
	double qtz = msg_real_pose_.orientation.z;
	double qtw = msg_real_pose_.orientation.w;

	tf::Quaternion q(qtx, qty, qtz, qtw);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	real_pose_(3) = roll;
	real_pose_(4) = pitch;
	real_pose_(5) = yaw;

}


void DSMotionGenerator::ComputeDesiredVelocity() {

	mutex_.lock();

	desired_velocity_ = SED_GMM_->getVelocity(real_pose_);

	// ROS_INFO_STREAM(real_pose_);

	// ROS_WARN_STREAM(desired_velocity_);


	msg_desired_velocity_.linear.x  = desired_velocity_(0);
	msg_desired_velocity_.linear.y  = desired_velocity_(1);
	msg_desired_velocity_.linear.z  = desired_velocity_(2);
	msg_desired_velocity_.angular.x = desired_velocity_(3);
	msg_desired_velocity_.angular.y = desired_velocity_(4);
	msg_desired_velocity_.angular.z = desired_velocity_(5);

	mutex_.unlock();

}


void DSMotionGenerator::PublishDesiredVelocity() {

	pub_desired_twist_.publish(msg_desired_velocity_);

}
