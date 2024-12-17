#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "rosgraph_msgs/Clock.h"
#include <boost/numeric/odeint.hpp>

typedef std::vector<double> state_type;

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
#define NAME_OF_THIS_NODE "node_example"
#define DT 0.15/10 //temporary, in case it will be changed by launch parameters 
#define X10 0.0
#define X20 0.0
#define X30 0.0
#define X40 0.0
#define X50 0.0
#define TA  0.15

class node_sim
{
  private: 
    ros::NodeHandle Handle; 
    ros::Subscriber sim_subscriber;
    ros::Publisher simPoseStamped_publisher;
    ros::Publisher simVel_publisher;
    ros::Publisher time_publisher;
    void sub_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void PeriodicTask(void);
    void Simulator_Step(void);
    std::string node_name;
    // SIMULATOR ZONE
    double simU_v_cmd, simU_omega_cmd;
    double simY_x, simY_y, simY_theta, simY_v, simY_omega;
    double sim_t, sim_dt;
    double Ta; //Time constant robot

    double Ts; // sample time
	
	double simY_xp, simY_yp; //feedback linearization state vars
	double xr,yr; //feedback linearization parameters

    int freq_multiplier;
    double endTime; //simulation running time
    state_type sim_state;
    boost::numeric::odeint::runge_kutta_dopri5 < state_type > stepper;
    void simulator_ode(const state_type &sim_state, state_type &sim_dstate, double t);
  
  
    
    
  public:
    double subtick; 
    void setInitialState(double x, double y,double theta);
    // void setModelParams(double Ta);
    void Prepare(void);
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};






void node_sim::Prepare(void)
{

	double Ts = DT;		 // sampling time
	double init_x,init_y,init_theta,init_v,init_omega;  // state initialization parameters

	node_name  = ros::this_node::getName();
	
	Handle.getParam(node_name+"/Ta", Ta);
	Handle.getParam(node_name+"/Ts", Ts);
	Handle.getParam(node_name+"/freq_multiplier", freq_multiplier);
	Handle.getParam(node_name+"/endTime", endTime);

	Handle.getParam(node_name+"/x0", init_x);
	Handle.getParam(node_name+"/y0", init_y);
	Handle.getParam(node_name+"/theta0", init_theta);
	Handle.getParam(node_name+"/v0", init_v);
	Handle.getParam(node_name+"/omega0", init_omega);

	Handle.getParam("/xr", xr);
	Handle.getParam("/yr", yr);	
	sim_subscriber = Handle.subscribe("/cmd", 3, &node_sim::sub_callback, this);
 	simPoseStamped_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/state", 5);
	time_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 10);

	setInitialState(init_x,init_y,init_theta);
	
	sim_t = 0.0;
	simU_v_cmd = 0.0;  
	simU_omega_cmd = 0.0;
	subtick = Ts/freq_multiplier;
	rosgraph_msgs::Clock clockMsg;

	clockMsg.clock = ros::Time(sim_t);
	time_publisher.publish(clockMsg);
	ROS_INFO("%s: Simulator Node ready to run.", node_name.c_str());
	
}

void node_sim::setInitialState(double x, double y,double theta){
	sim_state.resize(3); // specify size of state
    sim_state[0] = x;
    sim_state[1] = y;	
	sim_state[2] = theta;	
	ROS_INFO("%s: state init to:  %fm  %fm %frad",node_name.c_str(),x,y,theta);
}

void node_sim::RunPeriodically(float Period)
{	
	
	ros::WallRate  LoopRate(1.0/Period);
	ROS_INFO("%s: running periodically (T=%.2fs, f=%.2fHz).", node_name.c_str(), Period, 1.0/Period);
	while (ros::ok()) 
	{
		ros::spinOnce(); 
		PeriodicTask(); 
		LoopRate.sleep();
	}
}


void node_sim::Shutdown(void)
{
	ROS_INFO("%s: shutting down.", node_name.c_str());
	ros::shutdown();

}


void node_sim::sub_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	simU_v_cmd = msg->data[1];
	simU_omega_cmd = msg->data[2];
}

void node_sim::PeriodicTask(void)
{
	simY_x = sim_state[0];
	simY_y = sim_state[1]; 
	simY_theta = sim_state[2];
	simY_xp = simY_x + xr * cos(simY_theta) - yr * sin(simY_theta) ;
	simY_yp = simY_y + xr * sin(simY_theta) + yr * cos(simY_theta) ;


	rosgraph_msgs::Clock clockMsg;
	clockMsg.clock = ros::Time(sim_t);
	time_publisher.publish(clockMsg);

	std_msgs::Float64MultiArray msg;
	msg.data.resize(4);
	msg.data[0] = ros::Time::now().toSec();
	msg.data[1] = simY_x;
	msg.data[2] = simY_y;
	msg.data[3] = simY_theta;
	simPoseStamped_publisher.publish(msg);
	
	Simulator_Step();
	sim_t += subtick;
	
}

void node_sim::Simulator_Step(void)
{
	stepper.do_step(std::bind(&node_sim::simulator_ode, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), sim_state, sim_t, subtick);
}

void node_sim::simulator_ode(const state_type &state, state_type &dstate, double t)
{
    // Actual state
    const double sx = state[0]; 
    const double sy = state[1]; 
	const double stheta = state[2]; 

    double sv = simU_v_cmd; 
	double somega = simU_omega_cmd; 

    dstate[0] = cos(stheta)*sv;
    dstate[1] = sin(stheta)*sv;
	dstate[2] = somega;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE); 
  node_sim node;

  node.Prepare();
  
  node.RunPeriodically(node.subtick);
   
  node.Shutdown();
  
  return (0);
}
