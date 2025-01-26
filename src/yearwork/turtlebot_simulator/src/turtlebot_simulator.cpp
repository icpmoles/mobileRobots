#include "turtlebot_simulator.h"

#define X10 0.0
#define X20 0.0
#define X30 0.0
#define X40 0.0
#define X50 0.0
#define TA 0.15

void node_sim::Prepare(void)
{

	// double Ta;  		 // turtlebot time constant
	// double Ts = DT;										   // sampling time
	double init_x, init_y, init_theta, init_v, init_omega; // state initialization parameters

	node_name = ros::this_node::getName();

	GPMACRO(Ta);
	GPMACRO(tick);
	// GPMACRO(freq_multiplier);
	GPMACRO(endTime);
	GPMACRO(init_x);
	GPMACRO(init_y);
	GPMACRO(init_theta);
	GPMACRO(init_v);
	GPMACRO(init_omega);

	sim_subscriber = Handle.subscribe("/cmd", 1, &node_sim::sub_callback, this);
	simPoseStamped_publisher = Handle.advertise<geometry_msgs::PoseStamped>("/state", 1);
	simVel_publisher = Handle.advertise<geometry_msgs::TwistStamped>("/tb_vel", 1);
	time_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

	setInitialState(init_x, init_y, init_theta, init_v, init_omega);

	/* Node variable initialization */
	// default value if they haven't been received already
	sim_t = 0.0;
	simU_v_cmd = 0.0;
	simU_omega_cmd = 0.0;
	
	// // time pub
	rosgraph_msgs::Clock clockMsg;
	// what's the new time after running the simulation?

	clockMsg.clock = ros::Time(sim_t);
	// broadcasts first clock msg
	time_publisher.publish(clockMsg);
	ros::spinOnce();
	ROS_INFO("%s: Simulator Node ready to run for %f sec.", node_name.c_str(), endTime);
}

void node_sim::setInitialState(double x, double y, double theta, double v, double omega)
{
	sim_state.resize(5); // specify size of state
	sim_state[0] = x;
	sim_state[1] = y;
	sim_state[2] = theta;
	sim_state[3] = v;
	sim_state[4] = omega;
	ROS_INFO("%s: state init to:  %fm  %fm %frad\n%fm/s %frad/s", node_name.c_str(), x, y, theta, v, omega);
}

void node_sim::RunPeriodically(float Period)
{

	// ros::Rate  LoopRate(1.0/Period);
	ros::WallRate LoopRate(1.0 / Period);
	
	// Awaits for other nodes before starting the loop
	ROS_INFO("%s: waiting for other nodes.", node_name.c_str());
	sleep(1);
	ROS_INFO("%s: running periodically (T=%.2fs, f=%.2fHz).", node_name.c_str(), Period, 1.0 / Period);

	while (ros::ok())
	{
		ros::spinOnce();
		PeriodicTask();
		LoopRate.sleep();
	}
}

void node_sim::Shutdown(void)
{
	ROS_INFO("%s: shutting down.", node_name.c_str());
	ros::shutdown();
}

void node_sim::sub_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	// ROS_INFO("%s: received velocity/turn comand: %f %f ", node_name.c_str(), msg->linear.x, msg->angular.z);
	/* Receive data from the topic */
	simU_v_cmd = msg->twist.linear.x;
	simU_omega_cmd = msg->twist.angular.z;
	sent_time = msg->header.stamp;
	
}

void node_sim::PeriodicTask(void)
{
	if (ros::Time::now().toSec() > endTime)
	{
		Shutdown();
	}
	// elaboarate simulation values for more descriptive names
	simY_x = sim_state[0];
	simY_y = sim_state[1];
	simY_theta = sim_state[2];
	simY_v = sim_state[3];
	simY_omega = sim_state[4];

	rosgraph_msgs::Clock clockMsg;
	clockMsg.clock = ros::Time(sim_t);
	time_publisher.publish(clockMsg);

	geometry_msgs::PoseStamped poseMsg;
	
	poseMsg.header.stamp = ros::Time::now(); // - sent_time;
	poseMsg.pose.position.x = simY_x;
	poseMsg.pose.position.y = simY_y;
	poseMsg.pose.orientation.w = cos(simY_theta/2);
	poseMsg.pose.orientation.z = sin(simY_theta/2);
	simPoseStamped_publisher.publish(poseMsg);


	geometry_msgs::TwistStamped TwistStampedMsg;
	TwistStampedMsg.twist.linear.x = simY_v;
	TwistStampedMsg.twist.angular.z = simY_omega;
	simVel_publisher.publish(TwistStampedMsg);

	Simulator_Step();
	sim_t += tick;
}

void node_sim::Simulator_Step(void)
{
	stepper.do_step( std::bind(
			&node_sim::simulator_ode, 
			this, 
			std::placeholders::_1, 
			std::placeholders::_2, 
			std::placeholders::_3
			), 
		sim_state, 
		sim_t, 
		tick
	);
}

void node_sim::simulator_ode(const state_type &state, state_type &dstate, double t)
{
	// Actual state
	const double sx = state[0];
	const double sy = state[1];
	const double stheta = state[2];
	const double sv = state[3];
	const double somega = state[4];

	// Model equations of a unicycle with dynamics
	dstate[0] = cos(stheta) * sv;
	dstate[1] = sin(stheta) * sv;
	dstate[2] = somega;
	dstate[3] = (simU_v_cmd - sv) / Ta;
	dstate[4] = (simU_omega_cmd - somega) / Ta;
}