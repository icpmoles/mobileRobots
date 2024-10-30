#include "node_controller.h"

void node_contr::Prepare(void) // Janitor tasks
{

	/* Retrieve parameters from ROS parameter server */
	std::string kp_path,ki_path,kd_path,run_period_name;

	// PID params get

	run_period_name = ros::this_node::getName()+"/run_period";
	kp_path = ros::this_node::getName()+"/kp";
	ki_path = ros::this_node::getName()+"/ki";
	kd_path = ros::this_node::getName()+"/kd";
	Handle.getParam(run_period_name, RunPeriod);
	// FullParamName is a path
	// RunPeriod is where it stores the parameter

	// Initialize 
	u_act   = 0.0;
    uI_prev = 0.0;
    y_act   = 0.0;
    ysp_act = 0.0;

	if (true == Handle.getParam(run_period_name, RunPeriod))
	{
		ROS_INFO("Node %s: retrieved parameter %s.",
				ros::this_node::getName().c_str(), run_period_name.c_str());

		// FullParamName.c_str() = some functions prefer a "C style" string
		// Some functions prefer it this way
	}
	else
	{
		ROS_ERROR("Node %s: unable to retrieve parameter %s.",
				ros::this_node::getName().c_str(), run_period_name.c_str());
	}

	/* ROS topics */
	// create sub/pub 
	y_subscriber = Handle.subscribe("/simulation_output", 3, &node_contr::ControllerCallback, this);
	
	ROS_INFO("PID: listening to /simulation_output ");
	// "/topic1",		topic name
	// 1,  				buffer size. 1 = as real time as possible.
	// &node_contr::ControllerCallback, 
	// function callback: what gets executed when a msg is received
	// this, 			pointer to the object of the class
	// when a callback is implemented in an object way
	// it needs to know the pointer to the node handle
 	control_publisher = Handle.advertise<std_msgs::Float64>("/controller_cmd", 10);
	
	ROS_INFO("PID: advertising to /controller_cmd ");

	// std_msgs::Float64,  type of msg we are advertising
	// "/topic2", 			topic name
	// 1: buffer size, like for subscriber


	/* Node variable initialization */
	y_act = 0.0;  // default value if it hasn't been received already

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void node_contr::RunPeriodically(float Period)
{	

	//
	ros::Rate LoopRate(1.0/Period);

	ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);


	// infinite cycle
	// ros::ok always true unless: 
	//ctrl+c from the terminal 
	//or call/receives a Ros:kill command
	while (ros::ok()) 
	{
		// PeriodicTask(); // tasks I actually do...
		// ROS_INFO("PID Loop");
		ros::spinOnce(); 
		// after you completed your little tasks,
		// execute eventual callbacks that you received in the meanwhile
		usleep(1000);
		// LoopRate.sleep();
		// sleep until the next time slot
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