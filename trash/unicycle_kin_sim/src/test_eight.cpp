#include "unicycle_kin_sim/test_eight.h"
#include <cmath>

void test_eight::Prepare(void)
{
    RunPeriod = RUN_PERIOD_DEFAULT;

    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // run_period
    FullParamName = ros::this_node::getName()+"/run_period";
    if (false == Handle.getParam(FullParamName, RunPeriod))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // 8-shape trajectory parameters
    FullParamName = ros::this_node::getName()+"/a";
    if (false == Handle.getParam(FullParamName, a))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/T";
    if (false == Handle.getParam(FullParamName, T))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/cmd", 1);

    /* Initialize node state */
    linear_velocity = angular_velocity = 0.0;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void test_eight::RunPeriodically(float Period)
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

void test_eight::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void test_eight::PeriodicTask(void)
{
    /* 8-shaped trajectory generation */
    xref    = a*std::sin(2*M_PI/T*ros::Time::now().toSec());
    dxref   = 2*M_PI/T*a*std::cos(2*M_PI/T*ros::Time::now().toSec());
    ddxref  = -std::pow(2*M_PI/T,2.0)*a*std::sin(2*M_PI/T*ros::Time::now().toSec());
    yref    = a*std::sin(2*M_PI/T*ros::Time::now().toSec())*std::cos(2*M_PI/T*ros::Time::now().toSec());
    dyref   = 2*M_PI/T*a*(std::pow(std::cos(2*M_PI/T*ros::Time::now().toSec()),2.0)-std::pow(std::sin(2*M_PI/T*ros::Time::now().toSec()),2.0));
    ddyref  = -4.0*std::pow(2*M_PI/T,2.0)*a*std::sin(2*M_PI/T*ros::Time::now().toSec())*std::cos(2*M_PI/T*ros::Time::now().toSec());

    /* Vehicle commands */
    // Flatness transformation
    linear_velocity  = std::sqrt(std::pow(dxref,2.0)+std::pow(dyref,2.0));
    angular_velocity = (dxref*ddyref-dyref*ddxref)/(std::pow(dxref,2.0)+std::pow(dyref,2.0));

    /* Publishing vehicle commands (t, msg->data[0]; velocity, msg->data[1]; steer, msg->data[2]) */
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(ros::Time::now().toSec());
    msg.data.push_back(linear_velocity);
    msg.data.push_back(angular_velocity);
    vehicleCommand_publisher.publish(msg);
}
