#include "hello_world/node_example.h"


void node_example::Prepare(void) // Janitor tasks
{
	 RunPeriod = RUN_PERIOD_DEFAULT;

	/* Retrieve parameters from ROS parameter server */
	std::string FullParamName;

	// run_period
	FullParamName = ros::this_node::getName()+"/run_period";


	// Handle.getParam(FullParamName, RunPeriod)
	// FullParamName is a path
	// RunPeriod is where it stores the parameter

	if (true == Handle.getParam(FullParamName, RunPeriod))
	{
		ROS_INFO("Node %s: retrieved parameter %s.",
				ros::this_node::getName().c_str(), FullParamName.c_str());

		// FullParamName.c_str() = some functions prefer a "C style" string
		// Some functions prefer it this way
	}
	else
	{
		ROS_ERROR("Node %s: unable to retrieve parameter %s.",
				ros::this_node::getName().c_str(), FullParamName.c_str());
	}

	/* ROS topics */
	// create sub/pub 
	example_subscriber = Handle.subscribe("/topic1", 1, &node_example::topic1_MessageCallback, this);
	// "/topic1",		topic name
	// 1,  				buffer size. 1 = as real time as possible.
	// &node_example::topic1_MessageCallback, 
	// function callback: what gets executed when a msg is received
	// this, 			pointer to the object of the class
	// when a callback is implemented in an object way
	// it needs to know the pointer to the node handle
 	example_publisher = Handle.advertise<std_msgs::Float64>("/topic2", 1);

	// std_msgs::Float64,  type of msg we are advertising
	// "/topic2", 			topic name
	// 1: buffer size, like for subscriber


	/* Node variable initialization */
	topic1_data = 0.0;  // default value if it hasn't been received already

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void node_example::RunPeriodically(float Period)
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
		PeriodicTask(); // tasks I actually do...

		ros::spinOnce(); 
		// after you completed your little tasks,
		// execute eventual callbacks that you received in the meanwhile

		LoopRate.sleep();
		// sleep until the next time slot
	}
}


void node_example::Shutdown(void)
{
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
	// return 0 in the _core
	// put here janitorial functions for e.g. safe robot shutdown

}



// const std_msgs::Float64::ConstPtr& msg
// constant 
// & pointer
// to a message std_msgs::Float64 
void node_example::topic1_MessageCallback(const std_msgs::Float64::ConstPtr& msg)
{
	/* Receive data from the topic */
	topic1_data = msg->data;
}

void node_example::PeriodicTask(void)
{
	/* Put here the code related to the node task */
	/* Publish something on the topic */
    std_msgs::Float64 msg; // init msg
    msg.data = topic1_data; // loads data into it
	example_publisher.publish(msg); // publish it.
}

