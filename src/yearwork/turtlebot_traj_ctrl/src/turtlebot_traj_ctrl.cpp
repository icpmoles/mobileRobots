#include "turtlebot_traj_ctrl.h"

void node_contr::Prepare(void) // Janitor tasks
{

	std::string kp_path,ki_path,kd_path,run_period_name;

	kp_path = ros::this_node::getName()+"/kp";
	ki_path = ros::this_node::getName()+"/ki";
	kd_path = ros::this_node::getName()+"/kd";
	// Handle.getParam(run_period_name, RunPeriod);

	Handle.getParam("/tick_multiplier",multiplier);
	Handle.getParam("/subtick",subtick);

	Handle.getParam(ros::this_node::getName()+"/Kc",Kc);
	Handle.getParam(ros::this_node::getName()+"/Ti",Ti);
	
	Ts = subtick * multiplier;
	kp_path = ros::this_node::getName()+"/kp";
	a = Kc*Ts/Ti;
    b = Kc;
	// Initialize 
	u_act   = 0.0;
    uI_prev = 0.0;
    y_act   = 0.0;
    ysp_act = 0.0;
	y_act = 0.0;
	Handle.getParam("/equilibrium_angle",ysp_act);
	y_subscriber = Handle.subscribe("/simulation_output", 3, &node_contr::ControllerCallback, this);
	
	ROS_INFO("PID: listening to /simulation_output ");
 	control_publisher = Handle.advertise<std_msgs::Float64>("/controller_cmd", 10);
	
	ROS_INFO("PID: advertising to /controller_cmd ");

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void node_contr::RunPeriodically(float Period)
{	

	ros::Rate LoopRate(1.0/Period);
	ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
	while (ros::ok()) 
	{
		ros::spinOnce(); 
		LoopRate.sleep();
	}
}


void node_contr::Shutdown(void)
{
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
	// return 0 in the _core
	// put here janitorial functions for e.g. safe robot shutdown

}



// const std_msgs::Float64::ConstPtr& msg
// constant 
// & pointer
// to a message std_msgs::Float64 
void node_contr::ControllerCallback(const std_msgs::Float64::ConstPtr& msg)
{
	/* Receive data from the topic */
	ROS_INFO("PID: measurement acquired");
	y_act = msg->data;
	// double thetaD = msg->y;
	PID_Step();
}

void node_contr::PeriodicTask(void)
{
	/* Put here the code related to the node task */
	// /* Publish something on the topic */
    // std_msgs::Float64 msg; // init msg
    // msg.data = topic1_data; // loads data into it
	// control_publisher.publish(msg); // publish it.
}

void node_contr::PID_Step(void)
{
	ROS_INFO("Executing PID Step");
	// calculate PID controls
	double uI_act = uI_prev+a*(ysp_act-y_act);
    double uP_act = b*(ysp_act-y_act);
	u_act = uP_act+uI_act;

    // Update the state
    uI_prev = uI_act;
	// send Control signal
	std_msgs::Float64 msg; // init msg
    msg.data = u_act; // loads data into it
	control_publisher.publish(msg); // publish it.
	
	ROS_INFO("PID: u = %f, sp = %f, e = %f",u_act,ysp_act, ysp_act-y_act);
}