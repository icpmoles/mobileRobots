#include "unicycle_kin_sim/test_simple.h"

void test_simple::Prepare(void)
{
    RunPeriod = RUN_PERIOD_DEFAULT;

    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // run_period
    FullParamName = ros::this_node::getName()+"/run_period";
    if (false == Handle.getParam(FullParamName, RunPeriod))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // 8-shape trajectory parameters
    FullParamName = ros::this_node::getName()+"/V0";
    if (false == Handle.getParam(FullParamName, V0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/V1";
    if (false == Handle.getParam(FullParamName, V1))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/omega0";
    if (false == Handle.getParam(FullParamName, omega0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/omega1";
    if (false == Handle.getParam(FullParamName, omega1))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/T01";
    if (false == Handle.getParam(FullParamName, T01))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/robot_input", 1);

    /* Initialize node state */
    linear_velocity = angular_velocity = 0.0;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void test_simple::RunPeriodically(float Period)
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

void test_simple::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void test_simple::PeriodicTask(void)
{
    /* Vehicle commands */
    if (ros::Time::now().toSec()<=T01)
    {
        linear_velocity  = V0;
        angular_velocity = omega0;
    }
    else
    {
        linear_velocity  = V1;
        angular_velocity = omega1;
    }

    /* Publishing vehicle commands (t, msg->data[0]; velocity, msg->data[1]; steer, msg->data[2]) */
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(ros::Time::now().toSec());
    msg.data.push_back(linear_velocity);
    msg.data.push_back(angular_velocity);
    vehicleCommand_publisher.publish(msg);
}
