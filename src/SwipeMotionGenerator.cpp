#include "SwipeMotionGenerator.h"
// #include <tf/transform_datatypes.h>


SwipeMotionGenerator::SwipeMotionGenerator(ros::NodeHandle &n,
        double frequency,
        std::string input_topic_name,
        std::string output_topic_name,
        std::string output_filtered_topic_name,
        int SwipeDirection,
        double SwipeVelocity,
        double OrthogonalDamping,
        std::vector<double> SwipeTarget)
	: nh_(n),
	  loop_rate_(frequency),
	  input_topic_name_(input_topic_name),
	  output_topic_name_(output_topic_name),
	  output_filtered_topic_name_(output_filtered_topic_name),
	  dt_(1 / frequency),
	  Wn_(0),
	  SwipeDirection_(SwipeDirection),
	  SwipeVelocity_(SwipeVelocity),
	  OrthogonalDamping_(OrthogonalDamping),
	  OrthVelLim_(0.1),
	  SwipeTarget_(SwipeTarget),
	  SwipeVel_offset_(0),
	  Orth_damp_scaling_(1){

	ROS_INFO_STREAM("Motion generator node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


bool SwipeMotionGenerator::Init() {

	real_pose_.Resize(3);
	desired_velocity_.Resize(3);
	target_offset_.Resize(3);

	target_pose_.Resize(3);
	target_pose_(0) = SwipeTarget_[0];
	target_pose_(1) = SwipeTarget_[1];
	target_pose_(2) = SwipeTarget_[2];

	// initializing the filter
	CCDyn_filter_.reset (new CDDynamics(3, dt_, Wn_));

	// we should set the size automagically
	velLimits_.Resize(3);
	CCDyn_filter_->SetVelocityLimits(velLimits_);

	accLimits_.Resize(3);
	CCDyn_filter_->SetAccelLimits(accLimits_);


	MathLib::Vector initial(3);

	initial.Zero();

	CCDyn_filter_->SetState(initial);
	CCDyn_filter_->SetTarget(initial);

	if (!InitializeROS()) {
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	return true;
}



bool SwipeMotionGenerator::InitializeROS() {

	sub_real_pose_ = nh_.subscribe( input_topic_name_ , 1000,
	                                &SwipeMotionGenerator::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
	pub_desired_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_name_, 1);
	pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::TwistStamped>(output_filtered_topic_name_, 1);

	pub_target_ = nh_.advertise<geometry_msgs::PointStamped>("DS/target", 1);
	pub_DesiredPath_ = nh_.advertise<nav_msgs::Path>("DS/DesiredPath", 1);

	msg_DesiredPath_.poses.resize(MAX_FRAME);

	dyn_rec_f_ = boost::bind(&SwipeMotionGenerator::DynCallback, this, _1, _2);
	dyn_rec_srv_.setCallback(dyn_rec_f_);



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


void SwipeMotionGenerator::Run() {

	while (nh_.ok()) {

		ComputeDesiredVelocity();

		PublishDesiredVelocity();

		PublishFuturePath();

		ros::spinOnce();

		loop_rate_.sleep();
	}
}

void SwipeMotionGenerator::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	msg_real_pose_ = *msg;

	real_pose_(0) = msg_real_pose_.position.x;
	real_pose_(1) = msg_real_pose_.position.y;
	real_pose_(2) = msg_real_pose_.position.z;

}


void SwipeMotionGenerator::ComputeDesiredVelocity() {

	mutex_.lock();

	MathLib::Vector pose = real_pose_ - target_pose_  - target_offset_;

	double x_vel = 0;
	double y_vel = 0;
	double z_vel = - OrthogonalDamping_ * Orth_damp_scaling_* pose(2);

	if (z_vel  > OrthVelLim_) {
		z_vel =  OrthVelLim_;
	} else if (z_vel < -OrthVelLim_) {
		z_vel = - OrthVelLim_;
	}


	if (pose(1) * SwipeDirection_ < 0) {
		y_vel = SwipeDirection_ * (SwipeVelocity_ + SwipeVel_offset_);
	}


	// y_vel /= 1 + std::max(0, z_vel);
	// std::cout << x_vel << "\t" << y_vel << "\t" << z_vel << std::endl;

	desired_velocity_(0) = x_vel;
	desired_velocity_(1) = y_vel;
	desired_velocity_(2) = z_vel;

	if (std::isnan(desired_velocity_.Norm2())) {
		ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
		desired_velocity_.Zero();
	}


	msg_desired_velocity_.header.stamp = ros::Time::now();
	msg_desired_velocity_.twist.linear.x  = desired_velocity_(0);
	msg_desired_velocity_.twist.linear.y  = desired_velocity_(1);
	msg_desired_velocity_.twist.linear.z  = desired_velocity_(2);

	msg_desired_velocity_.twist.angular.x = 0;
	msg_desired_velocity_.twist.angular.y = 0;
	msg_desired_velocity_.twist.angular.z = 0;

	CCDyn_filter_->SetTarget(desired_velocity_);
	CCDyn_filter_->Update();
	CCDyn_filter_->GetState(desired_velocity_filtered_);

	msg_desired_velocity_filtered_.header.stamp = ros::Time::now();
	msg_desired_velocity_filtered_.twist.linear.x  = desired_velocity_filtered_(0);
	msg_desired_velocity_filtered_.twist.linear.y  = desired_velocity_filtered_(1);
	msg_desired_velocity_filtered_.twist.linear.z  = desired_velocity_filtered_(2);

	msg_desired_velocity_filtered_.twist.angular.x = 0;
	msg_desired_velocity_filtered_.twist.angular.y = 0;
	msg_desired_velocity_filtered_.twist.angular.z = 0;


	mutex_.unlock();

}


void SwipeMotionGenerator::PublishDesiredVelocity() {

	pub_desired_twist_.publish(msg_desired_velocity_);
	pub_desired_twist_filtered_.publish(msg_desired_velocity_filtered_);

}


void SwipeMotionGenerator::DynCallback(ds_motion_generator::SWIPE_paramsConfig &config, uint32_t level) {

	double  Wn = config.Wn;

	ROS_INFO("Reconfigure request. Updatig the parameters ...");


	Wn_ = config.Wn;
	CCDyn_filter_->SetWn(Wn_);

	double vlim = config.fil_dx_lim;

	velLimits_(0) = vlim;
	velLimits_(1) = vlim;
	velLimits_(2) = vlim;

	CCDyn_filter_->SetVelocityLimits(velLimits_);


	double alim = config.fil_ddx_lim;

	accLimits_(0) = alim;
	accLimits_(1) = alim;
	accLimits_(2) = alim;

	CCDyn_filter_->SetAccelLimits(accLimits_);

	target_offset_(0) = config.offset_x;
	target_offset_(1) = config.offset_y;
	target_offset_(2) = config.offset_z;

	// scaling_factor_ = config.scaling;
	// ds_vel_limit_   = config.trimming;

	SwipeVel_offset_ = config.SwipeVel_offset;
	Orth_damp_scaling_ = config.Orth_damp_scaling;
	OrthVelLim_ = config.OrthVelLim;

}


void SwipeMotionGenerator::PublishFuturePath() {

	geometry_msgs::PointStamped msg;

	msg.header.frame_id = "world";
	msg.header.stamp = ros::Time::now();
	msg.point.x = real_pose_[0];
	msg.point.y = target_pose_[1] + target_offset_[1];
	msg.point.z = target_pose_[2] + target_offset_[2];

	pub_target_.publish(msg);

	// create a temporary message


	// setting the header of the path
	msg_DesiredPath_.header.stamp = ros::Time::now();
	msg_DesiredPath_.header.frame_id = "world";




	MathLib::Vector simulated_pose = real_pose_;
	MathLib::Vector simulated_vel;
	simulated_vel.Resize(3);

	for (int frame = 0; frame < MAX_FRAME; frame++)
	{



	MathLib::Vector pose = simulated_pose - target_pose_  - target_offset_;


	simulated_vel(0) = 0;
	simulated_vel(1) = 0;
	simulated_vel(2) = - OrthogonalDamping_ * Orth_damp_scaling_* pose(2);


	if (simulated_vel(2)  > OrthVelLim_) {
		simulated_vel(2) =  OrthVelLim_;
	} else if (simulated_vel(2) < -OrthVelLim_) {
		simulated_vel(2) = - OrthVelLim_;
	}


	if (pose(1) * SwipeDirection_ < 0) {
		simulated_vel(1) = SwipeDirection_ * (SwipeVelocity_ + SwipeVel_offset_);
	}



		simulated_pose[0] +=  simulated_vel[0] * dt_ * 20;
		simulated_pose[1] +=  simulated_vel[1] * dt_ * 20;
		simulated_pose[2] +=  simulated_vel[2] * dt_ * 20;

		msg_DesiredPath_.poses[frame].header.stamp = ros::Time::now();
		msg_DesiredPath_.poses[frame].header.frame_id = "world";
		msg_DesiredPath_.poses[frame].pose.position.x = simulated_pose[0];
		msg_DesiredPath_.poses[frame].pose.position.y = simulated_pose[1];
		msg_DesiredPath_.poses[frame].pose.position.z = simulated_pose[2];

		pub_DesiredPath_.publish(msg_DesiredPath_);


	}


}
