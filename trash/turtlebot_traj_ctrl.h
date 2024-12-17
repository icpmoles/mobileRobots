#ifndef NODE_EXAMPLE_H_
#define NODE_EXAMPLE_H_

#include "ros/ros.h"
#include "turtlebot_traj_ctrl_PI.h"

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "tb_control_node"

#include "std_msgs/Float64.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

// i want to send float64 msgs between the example nodes
 
class node_contr
{
  private: 
    ros::NodeHandle Handle; 
    ros::Subscriber y_subscriber;
    ros::Publisher control_publisher;
    /* ROS topic callbacks */
    void ControllerCallback(const geometry_msgs::Pose::ConstPtr& msg);
 
    /* Node periodic task */
    void PeriodicTask(void);
    void PID_Step(void);
    
    /* Node state variables */

    // // PID state variables
    // double topic1_data;
    // double u_act, uI_prev, y_act, ysp_act;
    // double a, b;
    // double theta, thetaD;
    // double Kc, Ti, Ts;
    // int multiplier;
    PID PIDx, PIDy; // le PID: initialized with the same parameter
    
  public:
    double subtick; 
    // we want to use it to pass it to the RunPeriodically in the _core.cpp
    // we make it public 
    

    // functions stubs
    void Prepare(void);
    

    // runs the periodic loop inside
    // which then calls the PeriodicTask callback
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* NODE_EXAMPLE_H_ */