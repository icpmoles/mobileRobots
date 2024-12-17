#include "unicycle_kin_sim/unicycle_kin_sim.h"

#include <unistd.h>


void unicycle_kin_sim::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // Simulator parameters
    FullParamName = ros::this_node::getName()+"/dt";
    if (false == Handle.getParam(FullParamName, dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Vehicle initial state
    FullParamName = ros::this_node::getName()+"/x0";
    if (false == Handle.getParam(FullParamName, x0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/y0";
    if (false == Handle.getParam(FullParamName, y0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/theta0";
    if (false == Handle.getParam(FullParamName, theta0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    vehicleCommand_subscriber = Handle.subscribe("/cmd", 1, &unicycle_kin_sim::vehicleCommand_MessageCallback, this);
    vehicleState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/state", 1);
    clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

    /* Create simulator class */
    simulator = new unicycle_kin_ode(dt);

    /* Initialize simulator class */
    simulator->setInitialState(x0, y0, theta0);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void unicycle_kin_sim::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait other nodes start
    sleep(1.0);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        usleep(1000);
    }
}

void unicycle_kin_sim::Shutdown(void)
{
    // Delete ode object
    delete simulator;

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void unicycle_kin_sim::vehicleCommand_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // Input command: t, msg->data[0]; linear velocity, msg->data[1]; angular velocity, msg->data[2]
    /*  Set vehicle commands */
    simulator->setReferenceCommands(msg->data.at(1), msg->data.at(2));
}

void unicycle_kin_sim::PeriodicTask(void)
{
    /*  Integrate the model */
    simulator->integrate();

    /*  Extract measurement from simulator */
    double x, y, theta;
    simulator->getPoseStamped(x, y, theta);

    double linvelocity_act, angvelocity_act;
    simulator->getCommands(linvelocity_act, angvelocity_act);

    double time;
    simulator->getTime(time);

    /*  Print simulation time every 5 sec */
    if (std::fabs(std::fmod(time,5.0)) < 1.0e-3)
    {
        ROS_INFO("Simulator time: %d seconds", (int) time);
    }

    /*  Publish vehicle state */
    std_msgs::Float64MultiArray vehicleStateMsg;
    vehicleStateMsg.data.push_back(time);
    vehicleStateMsg.data.push_back(x);
    vehicleStateMsg.data.push_back(y);
    vehicleStateMsg.data.push_back(theta);
    vehicleStateMsg.data.push_back(linvelocity_act);
    vehicleStateMsg.data.push_back(angvelocity_act);
    vehicleState_publisher.publish(vehicleStateMsg);

    /*  Publish clock */
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = ros::Time(time);
    clock_publisher.publish(clockMsg);
}
