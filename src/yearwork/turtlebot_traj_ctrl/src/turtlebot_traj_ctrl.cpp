#ifndef MACRO_HELPER
#define MACRO_HELPER

#define GPMACRO(pname)                                                                                     \
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
#include "geometry_msgs/Vector3Stamped.h"

class node
{
private:
	ros::NodeHandle Handle;
	// ros::Subscriber example_subscriber;
	ros::Subscriber feedback_subscriber;
	ros::Publisher cmd_pub, sp_pub, p_pub, error_pub;

	void tb_MessageCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

	// y^ = set point
	double xp_dot; // x component of feedforward
	double yp_dot; // y component of feedforward
	double xp_;    // x component of trajectory
	double yp_;	   // y component of trajectory

	// states of the robot
	double theta_s; // yaw_state of the robot
	double x_s;		// x component of COG_state
	double y_s;		// y component of COG_state
	double yp_s;	// y component of P_state
	double xp_s; 	// x component of P_state

	// feedback linearization parameters
	double xr;		// eta (longitudinal component of lookahead)
	double yr;		// lateral component of lookahead		

	// trajectory parameters for circle/eight
	double a;		// trajectory amplitude
	double T;		// trajectory lap time

	std::string node_name;

	double pid_kc, pid_ti; // PID parameters
	
	double wait;			// Waits for n seconds before generating a trajectory
	PID PIDx, PIDy; // le PID: initialized with the same parameter

public:
	double samplingperiod;
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
	GPMACRO(samplingperiod);
	GPMACRO(T);
	GPMACRO(a);
	GPMACRO(xr);
	GPMACRO(yr);
	GPMACRO(wait);
	// GPMACRO(shiftf);
	// GPMACRO(R);
	feedback_subscriber = Handle.subscribe("/state", 1, &node::tb_MessageCallback, this);
	cmd_pub = Handle.advertise<geometry_msgs::TwistStamped>("/cmd", 1); // cmd publisher
	sp_pub = Handle.advertise<geometry_msgs::PoseStamped>("/setpoint", 1); // setpoint publisher
	p_pub = Handle.advertise<geometry_msgs::PoseStamped>("/state_p", 1);	// state from feedback linearization publisher
	error_pub = Handle.advertise<geometry_msgs::Vector3Stamped>("/error", 1);	//error publisher
	// theta_s = 0.0;

	double pid_ts = samplingperiod;
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
	ROS_INFO("Node %s: PI params KC: %f KI: %f  TS: %f", ros::this_node::getName().c_str(), pid_kc, pid_ti, samplingperiod);

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
	ros::Time now = ros::Time::now();
	double realtime = now.toSec();
	
	double t;
	double v;		// velocity cmd
	double omega; 	// twist cmd


	// In case plotjuggler doesn't start fast enough we wait n seconds and start everything with
	// a time shifted backwards t = (wallclock - shift)
	if (realtime >= wait) { 
		t = realtime - wait;
	
		double phi = 2 * 3.14 / T;
		 xp_dot = a * phi * cos(phi * t);
		 yp_dot = a * phi * cos(2 * phi * t);
		 xp_ = a * sin(phi * t);
		 yp_ = a * sin(phi * t) * cos(phi * t);
		// COONTROL FEEDBACK
		PIDx.setMeasurement(xp_s);
		PIDy.setMeasurement(yp_s);
		PIDx.setReference(xp_);
		PIDy.setReference(yp_);

		PIDx.execute();
		PIDy.execute();
		// FEED FORWARD
		double vx = PIDx.getControl() + xp_dot; // x component of V_ vector
		double vy = PIDy.getControl() + yp_dot;	// y component of V_ vector

		// FEEDBACK LINEARIZATION
		v = (cos(theta_s) - yr * sin(theta_s) / xr) * vx + (sin(theta_s) + yr * cos(theta_s) / xr) * vy; // i component of V_ vector
		omega = (-sin(theta_s) * vx + cos(theta_s) * vy) / xr;											// "j" component of V_ vector

		}  else {// else do nothing
			v,omega=0;
			yp_ = 0;
			xp_ = 0;

		}
		
		// LOOKAHEAD ESTIMATION
		xp_s = x_s + xr * cos(theta_s) - yr * sin(theta_s); //estimated x of P
		yp_s = y_s + xr * sin(theta_s) + yr * cos(theta_s); //estimated y of P

	
		// Publishes velocity/rotation comands
		geometry_msgs::TwistStamped msg;
		msg.header.stamp = now;
		msg.twist.linear.x = v;
		msg.twist.angular.z = omega;
		cmd_pub.publish(msg);

		// Publishes desired trajectory
		geometry_msgs::PoseStamped sp_msg;
		sp_msg.header.stamp = now;
		sp_msg.pose.position.x = xp_;
		sp_msg.pose.position.y = yp_;
		sp_pub.publish(sp_msg);

		// Publishes state of the robot 
		geometry_msgs::PoseStamped p_msg;
		p_msg.header.stamp = now;
		p_msg.pose.position.x = xp_s;
		p_msg.pose.position.y = yp_s;
		p_pub.publish(p_msg);

		// Publishes error of the P 
		double ex_ = xp_s-xp_;
		double ey_ = yp_s-yp_;
		geometry_msgs::Vector3Stamped e_msg;
		e_msg.header.stamp = now;
		e_msg.vector.x = ex_;
		e_msg.vector.y = ey_;
		error_pub.publish(e_msg);

	
}

void node::tb_MessageCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	x_s = msg->pose.position.x;
	y_s = msg->pose.position.y;
	theta_s = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NAME_OF_THIS_NODE);
	node node_node;
	node_node.Prepare();

	node_node.RunPeriodically(node_node.samplingperiod);

	node_node.Shutdown();

	return (0);
}