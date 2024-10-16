#include "hello_world/node_example.h"


void node_example::Prepare(void)
{
	RunPeriod = RUN_PERIOD_DEFAULT;

	/* Retrieve parameters from ROS parameter server */
	std::string FullParamName;

	// run_period
	FullParamName = ros::this_node::getName()+"/run_period";

	if (true == Handle.getParam(FullParamName, RunPeriod))
	{
		ROS_INFO("Node %s: retrieved parameter %s.",
				ros::this_node::getName().c_str(), FullParamName.c_str());
	}
	else
	{
		ROS_ERROR("Node %s: unable to retrieve parameter %s.",
				ros::this_node::getName().c_str(), FullParamName.c_str());
	}

	/* ROS topics */
	example_subscriber = Handle.subscribe("/topic1", 1, &node_example::topic1_MessageCallback, this);
	example_publisher = Handle.advertise<std_msgs::Float64>("/topic2", 1);

	/* Node variable initialization */
	topic1_data = 0.0;

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void node_example::RunPeriodically(float Period)
{
	ros::Rate LoopRate(1.0/Period);

	ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

	while (ros::ok())
	{
		PeriodicTask();

		ros::spinOnce();

		LoopRate.sleep();
	}
}


void node_example::Shutdown(void)
{
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());

}


void node_example::topic1_MessageCallback(const std_msgs::Float64::ConstPtr& msg)
{
	/* Receive data from the topic */
	topic1_data = msg->data;
}

void node_example::PeriodicTask(void)
{
	/* Put here the code related to the node task */
	
	/* Publish something on the topic */
    std_msgs::Float64 msg;
    msg.data = topic1_data;
	example_publisher.publish(msg);
}

