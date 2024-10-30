#ifndef NODE_EXAMPLE_H_
#define NODE_EXAMPLE_H_
#include <boost/numeric/odeint.hpp>

typedef std::vector<double> state_type;
#include "ros/ros.h"
#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
#define NAME_OF_THIS_NODE "node_example"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "rosgraph_msgs/Clock.h"
// i want to send float64 msgs between the example nodes
class node_sim
{
  private: 
    ros::NodeHandle Handle; //ROS Handle
    // initialized at ros::init
    // needs to be used if you call a function of the 
    // ROS client library
    
    /* ROS topics */
    ros::Subscriber sim_subscriber;
    ros::Publisher sim_publisher;
    ros::Publisher time_publisher;
    /* Parameters from ROS parameter server */
    // param_type ParamVar;
    // where to store the parameters retrieved by the param server

    /* ROS topic callbacks */
    void sub_callback(const std_msgs::Float64::ConstPtr& msg);
 
    /* Node periodic task */
    void PeriodicTask(void);
    
    void Simulator_Step(bool multi);
    

    // SIMULATOR ZONE
    /* Node state variables */
    double sim_u,sim_y1,sim_y2;
    double sim_t, sim_dt;
    double sim_m, sim_l, sim_d;
    state_type sim_state;
    boost::numeric::odeint::runge_kutta_dopri5 < state_type > stepper;
    void simulator_ode(const state_type &sim_state, state_type &sim_dstate, double t);
    int iteration;
    
    
  public:
    double RunPeriod; 
  
    // we want to use it to pass it to the RunPeriodically in the _core.cpp
    // we make it public
    void setInitialState(double x1, double x2);
    void setModelParams(double m, double l, double d);
        // mass, length, friction coefficent 

    // void integrate();
    // void setInputValues(double u);
  
    void getState(double &x1, double &x2);
    void getTime(double &time); //get time after integration step


    // functions stubs
    void Prepare(void);
    

    // runs the periodic loop inside
    // which then calls the PeriodicTask callback
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* NODE_EXAMPLE_H_ */