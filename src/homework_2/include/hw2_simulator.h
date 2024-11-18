#ifndef NODE_EXAMPLE_H_
#define NODE_EXAMPLE_H_
#include <boost/numeric/odeint.hpp>

typedef std::vector<double> state_type;

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
#define NAME_OF_THIS_NODE "node_example"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
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
    ros::Publisher simPose_publisher;
    ros::Publisher simVel_publisher;
    ros::Publisher time_publisher;
    /* Parameters from ROS parameter server */
    // param_type ParamVar;
    // where to store the parameters retrieved by the param server

    /* ROS topic callbacks */
    void sub_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
 
    /* Node periodic task */
    void PeriodicTask(void);
    
    void Simulator_Step(void);
    
    std::string node_name;
    // SIMULATOR ZONE
    /* Node state variables */
    double simU_v_cmd, simU_omega_cmd;
    // double simX_x, simX_y, simX_theta, simX_v, simX_omega;
    double simY_x, simY_y, simY_theta, simY_v, simY_omega;
    double sim_t, sim_dt;
    double Ta; //Time constant robot

    double Ts; // sample time
    int freq_multiplier;
    double endTime; //simulation running time
    state_type sim_state;
    boost::numeric::odeint::runge_kutta_dopri5 < state_type > stepper;
    void simulator_ode(const state_type &sim_state, state_type &sim_dstate, double t);
  
  
    
    
  public:
    double subtick; 
  
    // we want to use it to pass it to the RunPeriodically in the _core.cpp
    // we make it public
    void setInitialState(double x, double y,double theta, double v, double omega);
    void setModelParams(double Ta);

    // void integrate();
    // void setInputValues(double u);
  
    // void getPose(double &x, double &y, double &theta);
    // void getTime(double &time); //get time after integration step
    // void getVel(double &v, double &omega);

    // functions stubs
    void Prepare(void);
    

    // runs the periodic loop inside
    // which then calls the PeriodicTask callback
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* NODE_EXAMPLE_H_ */