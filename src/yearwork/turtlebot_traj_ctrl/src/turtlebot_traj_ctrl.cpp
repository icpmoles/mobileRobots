#ifndef MACRO_HELPER
#define MACRO_HELPER

#define GPMACRO(pname)                                                                                    \
	if (false == Handle.getParam(ros::this_node::getName() + "/" + #pname, pname))                         \
	{                                                                                                      \
		ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), #pname); \
	}

#endif /* MACRO_HELPER */	


#include "ros/ros.h"
#include "turtlebot_traj_ctrl_PI.h"
// #include "std_msgs/Float64MultiArray.h"
#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "node_example"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"


class node
{
private:
	ros::NodeHandle Handle;
	ros::Subscriber example_subscriber;
	ros::Subscriber feedback_subscriber;
	ros::Publisher publisher,P_pub;

	void tb_MessageCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

	double xp_dot, yp_dot; // velocity of trajectory: feedforward
	double xp_, yp_;	   // cartesian coordinates of trajectory: Feed into PID

	double theta_s, x_s, y_s, yp_s, xp_s; // states of the robot
	double xr, yr;						  // feedback linearization parameters
	double a, T;						  // trajectory parameters for circle/eight
	std::string node_name;

	double pid_kc, pid_ti; // PID parameters
	PID PIDx, PIDy;		   // le PID: initialized with the same parameter

public:
	double refreshperiod;
	double RunPeriod;
	void Prepare(void);
	void PeriodicTask(void);
	void RunPeriodically(float Period);
	void Shutdown(void);
};

void node::Prepare(void)
{
	RunPeriod = RUN_PERIOD_DEFAULT;
	node_name = ros::this_node::getName();
	GPMACRO(pid_kc);
	GPMACRO(pid_ti);
	GPMACRO(refreshperiod);
	GPMACRO(T);
	GPMACRO(a);
	GPMACRO(xr);
	GPMACRO(yr);
	// GPMACRO(R);
	feedback_subscriber = Handle.subscribe("/state", 1, &node::tb_MessageCallback, this);
	publisher = Handle.advertise<geometry_msgs::TwistStamped>("/cmd", 1);
	P_pub = Handle.advertise<geometry_msgs::PoseStamped>("/setpoint", 1);

	theta_s = 0.0;

	double pid_ts = refreshperiod;
	// pid_a = pid_kc*pid_ts/pid_ti;
	// pid_b = pid_kc;

	PIDx.initialize(pid_kc, pid_ti, pid_ts);
	PIDy.initialize(pid_kc, pid_ti, pid_ts);
	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void node::RunPeriodically(float Period)
{
	ros::Rate LoopRate(1.0 / Period);

	ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0 / Period);
	ROS_INFO("Node %s: params xr: %f yr: %f R: %f T: %f", ros::this_node::getName().c_str(), xr, yr, a, T);
	ROS_INFO("Node %s: PI params KC: %f KI: %f  TS: %f", ros::this_node::getName().c_str(), pid_kc, pid_ti, refreshperiod);

	while (ros::ok())
	{
		node::PeriodicTask();
		ros::spinOnce();
		LoopRate.sleep();
	}
}

void node::Shutdown(void)
{
	// delete *PIDx;
	// delete *PIDy;
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void node::PeriodicTask(void)
{

	double t = ros::Time::now().toSec();
	double phi = 2 * 3.14 / T;

	// TRAJECTORY GENERATION

	xp_dot = a * phi * cos(phi * t);
	yp_dot = a * phi * cos( 2* phi * t);
	xp_ = a * sin(phi * t);
	yp_ = a * sin(phi * t) * cos(phi * t);


	// LOOKAHEAD ESTIMATION
	xp_s = x_s + xr * cos(theta_s) - yr * sin(theta_s);
	yp_s = y_s + xr * sin(theta_s) + yr * cos(theta_s);

	// COONTROL FEEDBACK
	PIDx.setMeasurement(xp_s);
	PIDy.setMeasurement(yp_s);
	PIDx.setReference(xp_);
	PIDy.setReference(yp_);

	PIDx.execute();
	PIDy.execute();
	// FEED FORWARD
	double vx = PIDx.getControl() + xp_dot;
	double vy = PIDy.getControl() + yp_dot;

	// double pidy_u_act = PIDy.u_act;
	// double pidx_u_act = PIDx.u_act;

	// FEEDBACK LINEARIZATION
	double v, omega;
	v = (cos(theta_s) - yr * sin(theta_s) / xr) * vx + (sin(theta_s) + yr * cos(theta_s) / xr) * vy;
	omega = (-sin(theta_s) * vx + cos(theta_s) * vy) / xr;

	// PUBLISHING
	//   std_msgs::Float64MultiArray msgtosend;
	//   msgtosend.data.resize(3);
	//   msgtosend.data[0] = t;
	//   msgtosend.data[1] = v;
	//   msgtosend.data[2] = omega;

	geometry_msgs::TwistStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.twist.linear.x = v;
	msg.twist.angular.z = omega;

	// ROS_INFO("Node %s: received theta: %f\n calculated v:%f w: %f", ros::this_node::getName().c_str(),theta,v,omega);
	publisher.publish(msg);


	geometry_msgs::PoseStamped p_msg;
	p_msg.pose.position.x = xp_;
	p_msg.pose.position.y = yp_;
	P_pub.publish(p_msg);
}

void node::tb_MessageCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	x_s = msg->pose.position.x;
	y_s = msg->pose.position.y;
	theta_s = atan2(msg->pose.orientation.z, msg->pose.orientation.w);

	// data aquisition
	// x   = msg->data[1];   //not really needed
	// y   = msg->data[2];   //not really needed
	//   theta = msg->data[3];
	// xp  = msg->data[4];
	// yp  = msg->data[5];
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NAME_OF_THIS_NODE);
	node node_node;
	node_node.Prepare();

	node_node.RunPeriodically(node_node.refreshperiod);

	node_node.Shutdown();

	return (0);
}