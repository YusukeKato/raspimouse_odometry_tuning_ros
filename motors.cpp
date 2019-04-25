#include <ros/ros.h>
#include <signal.h>
#include <ros/package.h> 
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Twist.h"
#include "raspimouse_ros_2/MotorFreqs.h"
#include "raspimouse_ros_2/TimedMotion.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <fstream>
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "raspimouse_odometry_tuning/CalibrationValue.h"

using namespace ros;

bool setPower(bool);
void setFreqs(int left, int right);

float forward_L = 1.0;
float forward_R = 1.0;
float back_L = 1.0;
float back_R = 1.0;
float right_L = 1.0;
float right_R = 1.0;
float left_L = 1.0;
float left_R = 1.0;

float pre_forward_R = 1.0;

void onSigint(int);
bool callbackOn(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
bool callbackOff(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
bool callbackTimedMotion(raspimouse_ros_2::TimedMotion::Request&, raspimouse_ros_2::TimedMotion::Response&);
void callbackRaw(const raspimouse_ros_2::MotorFreqs::ConstPtr& msg);
void callbackCmdvel(const geometry_msgs::Twist::ConstPtr& msg);
void callback9Axis(const sensor_msgs::Imu::ConstPtr& msg);

void callbackCV(const raspimouse_odometry_tuning::CalibrationValue::ConstPtr& msg);

bool is_on = false;
bool in_cmdvel = false;
Time last_cmdvel;
Time cur_time;
Time send_time;

geometry_msgs::Twist vel;
double odom_x,odom_y,odom_theta;

bool imu_flag = false;

// Calibration Values : -> setFreqs
//float caliValue_left = 1.0;
//float caliValue_right = 1.0;


bool setPower(bool on)
{
	std::ofstream ofs("/dev/rtmotoren0");
	if(not ofs.is_open())
		return false;

	ofs << (on ? '1' : '0') << std::endl;
	is_on = on;
	return true;
}

void setFreqs(int left, int right)
{
	//ROS_INFO("left:%d", left);
	//ROS_INFO("right:%d", right);

	if(abs(forward_R - pre_forward_R) > 0.01)
	{
		pre_forward_R = forward_R;
		odom_x = 0.0;
		odom_y = 0.0;
		odom_theta = 0.0;
	}

	std::ofstream ofsL("/dev/rtmotor_raw_l0");
	std::ofstream ofsR("/dev/rtmotor_raw_r0");
	if( (not ofsL.is_open()) or (not ofsR.is_open()) ){
		ROS_ERROR("Cannot open /dev/rtmotor_raw_{l,r}0");
		return;
	}

	int right2 = right;
	int left2 = left;

	if(right >= 0 && left >= 0) //forward
	{
		right2 = (int)round(right * forward_R);
		left2 = (int)round(left * forward_L);
	}
	else if(right < 0 && left < 0) //back
	{
		right2 = (int)round(right * back_R);
		left2 = (int)round(left * back_L);
	}
	else if(right < 0 && left >= 0) //right turn
	{
		right2 = (int)round(right * right_R);
		left2 = (int)round(left * right_L);
	}
	else if(right >= 0 && left < 0) //left turn
	{
		right2 = (int)round(right * left_R);
		left2 = (int)round(left * left_L);
	}

	if(left2 > 800) left2 = 800;
	else if(left2 < -800) left2 = -800;
	if(right2 > 800) right2 = 800;
	else if(right2 < -800) right2 = -800;

	ofsL << left2 << std::endl;
	ofsR << right2 << std::endl;
}

void onSigint(int sig)
{
	setPower(false);
	exit(0);
}

bool callbackOn(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
	if(not setPower(true))
		return false;

	response.message = "ON";
	response.success = true;
	return true;
}

bool callbackOff(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
	if(not setPower(false))
		return false;

	response.message = "OFF";
	response.success = true;
	return true;
}

bool callbackTimedMotion(raspimouse_ros_2::TimedMotion::Request& request, raspimouse_ros_2::TimedMotion::Response& response)
{
	if(not is_on){
		ROS_INFO("Motors are not enpowered");
		return false;
	}

	std::ofstream ofs("/dev/rtmotor0");
	if(not ofs.is_open()){
		ROS_ERROR("Cannot open /dev/rtmotor0");
		return false;
	}

	ofs << request.left_hz << ' ' << request.right_hz << ' '
		<< request.duration_ms << std::endl;

	response.success = true;
	return true;
}

void callbackRaw(const raspimouse_ros_2::MotorFreqs::ConstPtr& msg)
{
	setFreqs(msg->left_hz, msg->right_hz);
}

void callbackCmdvel(const geometry_msgs::Twist::ConstPtr& msg)
{
	vel.linear.x = msg->linear.x;

	if(!imu_flag)
		vel.angular.z = msg->angular.z;

	double forward_hz = 80000.0*msg->linear.x/(9*3.141592);
	double rot_hz = 400.0*msg->angular.z/3.141592;
	setFreqs((int)round(forward_hz-rot_hz), (int)round(forward_hz+rot_hz));
	in_cmdvel = true;
	last_cmdvel = Time::now();
}

void callback9Axis(const sensor_msgs::Imu::ConstPtr& msg)
{
	if(not imu_flag){
		ROS_INFO("9-axis sensor mode");
		imu_flag = true;
	}

	vel.angular.z = round(msg->angular_velocity.z * 20)/20; // cut less than 0.05[rad/s]
}

void callbackCV(const raspimouse_odometry_tuning::CalibrationValue::ConstPtr& msg)
{
	forward_R = msg->forward_R;
	forward_L = msg->forward_L;
	back_R = msg->back_R;
	back_L = msg->back_L;
	right_R = msg->right_R;
	right_L = msg->right_L;
	left_L = msg->left_L;
	left_L = msg->left_L;
}

// CallBack Calibration Values
//void callbackCVLeft(const std_msgs::Float64::ConstPtr& msg)
//{
//	caliValue_left = msg->data;
//	//ROS_INFO("sub_left=%lf", caliValue_left);
//}

//void callbackCVRight(const std_msgs::Float64::ConstPtr& msg)
//{
//	caliValue_right = msg->data;
//	//ROS_INFO("sub_right=%d", caliValue_right);
//}

nav_msgs::Odometry send_odom(void)
{
	cur_time = Time::now();

	double dt = cur_time.toSec() - send_time.toSec();
	odom_x += vel.linear.x * cos(odom_theta) * dt;
	odom_y += vel.linear.x * sin(odom_theta) * dt;

	odom_theta += vel.angular.z * dt;

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_theta);

	static tf2_ros::TransformBroadcaster br;

	geometry_msgs::TransformStamped odom_trans;
 	odom_trans.header.stamp = cur_time;
  	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

  	odom_trans.transform.translation.x = odom_x;
  	odom_trans.transform.translation.y = odom_y;
  	odom_trans.transform.translation.z = 0.0;

	tf2::Quaternion q;
	q.setRPY(0, 0, odom_theta);
	odom_trans.transform.rotation.x = q.x();
 	odom_trans.transform.rotation.y = q.y();
	odom_trans.transform.rotation.z = q.z();
	odom_trans.transform.rotation.w = q.w();

	br.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	odom.header.stamp = cur_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

	odom.pose.pose.position.x = odom_x;
	odom.pose.pose.position.y = odom_y;
	odom.pose.pose.position.z = 0.0;
  	odom_trans.transform.rotation = odom_quat;
	odom.pose.pose.orientation = odom_quat;

	odom.twist.twist.linear.x = vel.linear.x;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = vel.angular.z;

	send_time = cur_time;
	return odom;
}

int main(int argc, char **argv)
{
	setFreqs(0,0);

	init(argc,argv,"motors");
	NodeHandle n;

	std::string onoff;
	if(argc > 1)
		onoff = argv[1];

	setPower(onoff == "on");

	signal(SIGINT, onSigint);

	last_cmdvel = Time::now();
	cur_time = Time::now();
	send_time = Time::now();

	ServiceServer srv_on = n.advertiseService("motor_on", callbackOn);
	ServiceServer srv_off = n.advertiseService("motor_off", callbackOff);
	ServiceServer srv_tm = n.advertiseService("timed_motion", callbackTimedMotion); 
	
	Subscriber sub_raw = n.subscribe("motor_raw", 10, callbackRaw);
	Subscriber sub_cmdvel = n.subscribe("cmd_vel", 10, callbackCmdvel);
	Subscriber sub_9axis = n.subscribe("/imu/data_raw", 10, callback9Axis);

	// Subscriber Calibration Values
	Subscriber sub_cv = n.subscribe("calibration_value", 10, callbackCV);
	//Subscriber sub_cvLeft = n.subscribe("cv_left", 10, callbackCVLeft);
	//Subscriber sub_cvRight = n.subscribe("cv_right", 10, callbackCVRight);

	Publisher pub_odom = n.advertise<nav_msgs::Odometry>("odom", 1); //queue_size:10->1
	odom_x = 0.0;
	odom_y = 0.0;
	odom_theta = 0.0;

	send_time = Time::now();

	Rate loop_rate(90);
	while(ok()){
		if(in_cmdvel and Time::now().toSec() - last_cmdvel.toSec() >= 1.0)
			setFreqs(0,0);

		pub_odom.publish(send_odom());
		spinOnce();
		loop_rate.sleep();
	}
	
	exit(0);
}
