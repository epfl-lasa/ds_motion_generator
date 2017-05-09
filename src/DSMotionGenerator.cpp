#include "DSMotionGenerator.h"
#include <tf/transform_datatypes.h>




DSMotionGenerator::DSMotionGenerator(ros::NodeHandle &n,
                                     double frequency,
                                     int K_gmm,
                                     int dim,
                                     std::vector<double> Priors,
                                     std::vector<double> Mu,
                                     std::vector<double> Sigma,
                                     std::vector<double> attractor,
                                     std::string input_topic_name,
                                     std::string output_topic_name,
                                     std::string output_filtered_topic_name)
	: nh_(n),
	  loop_rate_(frequency),
	  K_gmm_(K_gmm),
	  dim_(dim),
	  Priors_(Priors),
	  Mu_(Mu),
	  Sigma_(Sigma),
	  attractor_(attractor),
	  input_topic_name_(input_topic_name),
	  output_topic_name_(output_topic_name),
	  output_filtered_topic_name_(output_filtered_topic_name),
	  dt_(1 / frequency),
	  Wn_(0),
	  scaling_factor_(1),
	  ds_vel_limit_(0.1) {

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

	if (attractor_.size() != 6) {
		ROS_ERROR_STREAM("InitializeDS: Please provide 6 elements for the attractor. It has " << attractor_.size() << " elements.");
		return false;
	}


	SED_GMM_.reset (new GMRDynamics(K_gmm_, dim_, dt_, Priors_, Mu_, Sigma_ ));
	SED_GMM_->initGMR(0, 2, 3, 5 );


	target_offset_.Resize(6);
	target_pose_.Resize(6);

	for (int i=0; i < attractor_.size(); i++){
		target_pose_(i) = attractor_[i];
	}

	// to do: the dimension for the filter should be equal to dimension of the output space.


	// initializing the filter
	CCDyn_filter_.reset (new CDDynamics(6, dt_, Wn_));

	// we should set the size automagically
	velLimits_.Resize(6);
	CCDyn_filter_->SetVelocityLimits(velLimits_);

	accLimits_.Resize(6);
	CCDyn_filter_->SetAccelLimits(accLimits_);


	MathLib::Vector initial(6);

	initial.Zero();

	CCDyn_filter_->SetState(initial);
	CCDyn_filter_->SetTarget(initial);


	return true;

}


bool DSMotionGenerator::InitializeROS() {

	sub_real_pose_ = nh_.subscribe( input_topic_name_ , 1000,
	                                &DSMotionGenerator::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
	pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>(output_topic_name_, 1);
	pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::Twist>(output_filtered_topic_name_, 1);


	dyn_rec_f_ = boost::bind(&DSMotionGenerator::DynCallback, this, _1, _2);
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


void DSMotionGenerator::Run() {

	while (nh_.ok()) {

		ComputeDesiredVelocity();

		PublishDesiredVelocity();

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

	desired_velocity_ = SED_GMM_->getVelocity(real_pose_ - target_pose_ - target_offset_);

	if(isnan(desired_velocity_.Norm2())){
		ROS_WARN_THROTTLE(1,"DS is generating NaN. Setting the output to zero.");
		desired_velocity_.Zero();		
	}

	desired_velocity_ = desired_velocity_ * scaling_factor_;

	if (desired_velocity_.Norm() > ds_vel_limit_){
		desired_velocity_ = desired_velocity_ / desired_velocity_.Norm() * ds_vel_limit_;
	}

	msg_desired_velocity_.linear.x  = desired_velocity_(0);
	msg_desired_velocity_.linear.y  = desired_velocity_(1);
	msg_desired_velocity_.linear.z  = desired_velocity_(2);
	msg_desired_velocity_.angular.x = desired_velocity_(3);
	msg_desired_velocity_.angular.y = desired_velocity_(4);
	msg_desired_velocity_.angular.z = desired_velocity_(5);

	CCDyn_filter_->SetTarget(desired_velocity_);
	CCDyn_filter_->Update();
	CCDyn_filter_->GetState(desired_velocity_filtered_);

	msg_desired_velocity_filtered_.linear.x  = desired_velocity_filtered_(0);
	msg_desired_velocity_filtered_.linear.y  = desired_velocity_filtered_(1);
	msg_desired_velocity_filtered_.linear.z  = desired_velocity_filtered_(2);
	msg_desired_velocity_filtered_.angular.x = desired_velocity_filtered_(3);
	msg_desired_velocity_filtered_.angular.y = desired_velocity_filtered_(4);
	msg_desired_velocity_filtered_.angular.z = desired_velocity_filtered_(5);


	mutex_.unlock();

}


void DSMotionGenerator::PublishDesiredVelocity() {

	pub_desired_twist_.publish(msg_desired_velocity_);
	pub_desired_twist_filtered_.publish(msg_desired_velocity_filtered_);


}


void DSMotionGenerator::DynCallback(ds_motion_generator::SED_paramsConfig &config, uint32_t level) {


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

	scaling_factor_ = config.scaling;
	ds_vel_limit_   = config.trimming;

}