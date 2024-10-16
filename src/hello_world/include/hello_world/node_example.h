#ifndef NODE_EXAMPLE_H_
#define NODE_EXAMPLE_H_

#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "node_example"

#include "std_msgs/Float64.h"
// i want to send float64 msgs between the example nodes
 
class node_example
{
  private: 
    ros::NodeHandle Handle; //ROS Handle
    // initialized at ros::init
    // needs to be used if you call a function of the 
    // ROS client library
    
    /* ROS topics */
    ros::Subscriber example_subscriber;
    ros::Publisher example_publisher;
    
    /* Parameters from ROS parameter server */
    // param_type ParamVar;
    // where to store the parameters retrieved by the param server

    /* ROS topic callbacks */
    void topic1_MessageCallback(const std_msgs::Float64::ConstPtr& msg);
 
    /* Node periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    double topic1_data;
  
    
  public:
    double RunPeriod; 
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