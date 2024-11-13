#include "node_simulator.h"

#define DT 0.001
#define M 10.0
#define L 1.0
#define D 4.0
#define X10 0.0
#define X20 0.0
#define NUM_STEP 10000



void node_sim::Prepare(void) // Janitor tasks
{

	/* Retrieve parameters from ROS parameter server */
	std::string m_name,l_name,d_name,dt_name,run_period_name,node_name;

	node_name  = ros::this_node::getName();
	run_period_name = node_name+"/run_period";
	m_name = node_name+"/m";
	
	d_name = node_name+"/d";
	
	l_name = node_name+"/l";
	dt_name = node_name+"/dt";
	Handle.getParam(m_name, sim_m);
	Handle.getParam(l_name, sim_l);
	Handle.getParam(d_name, sim_d);
	Handle.getParam("/tick_multiplier",multiplier);
	Handle.getParam("/subtick",subtick);
	Handle.getParam(node_name+"/initial_angle",initial_angle);
	Handle.getParam(node_name+"/initial_angular_vel",initial_angular_vel);

	sim_dt = subtick*multiplier;

	sim_subscriber = Handle.subscribe("/controller_cmd", 3, &node_sim::sub_callback, this);
 	sim_publisher = Handle.advertise<std_msgs::Float64>("/simulation_output", 5);
	time_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 10);

	sim_state.resize(2); // specify size of state
    sim_state[0] = 0.0;
    sim_state[1] = 0.0;	

	setInitialState(initial_angle,initial_angular_vel);
	
	// TIME starts from 1s instead of from 0s to avoid a bug in ROS somewhere
	// https://github.com/ros-simulation/stage_ros/blob/c3abc19b9e9713fe450597ca33b6f5399d1691ca/src/stageros.cpp#L426
	sim_t = 1.0;
	iteration = 0;
    

	/* Node variable initialization */
	sim_u = 0.0;  // default value if it hasn't been received already
		// // time pub
	rosgraph_msgs::Clock clockMsg;
	// what's the new time after running the simulation?

	clockMsg.clock = ros::Time(0.0);
	time_publisher.publish(clockMsg);
 	
	ROS_INFO("Sim Node %s ready to run.", ros::this_node::getName().c_str());
	
}

void node_sim::setInitialState(double x1, double x2){
	sim_state[0] = x1;
    sim_state[1] = x2;
	ROS_INFO("Sim: state init to:  %f %f",x1,x2);
}

void node_sim::RunPeriodically(float Period)
{	

	//
	ros::WallRate  LoopRate(1.0/Period);

	ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

	
	while (ros::ok()) 
	{
		
		ros::spinOnce(); 
		PeriodicTask(); 
		LoopRate.sleep();
	}
}


void node_sim::Shutdown(void)
{
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());

}



// const std_msgs::Float64::ConstPtr& msg
// constant 
// & pointer
// to a message std_msgs::Float64 
void node_sim::sub_callback(const std_msgs::Float64::ConstPtr& msg)
{
	
	ROS_INFO("Sim: control u received");
	/* Receive data from the topic */
	sim_u = msg->data;

	Simulator_Step(false);
	ROS_INFO("Sim: end of callback");
}

void node_sim::PeriodicTask(void)
{
	if (iteration == NUM_STEP) {
		ros::shutdown();
	}
    sim_t += (subtick);
	// starts simulations
	/* Put here the code related to the node task */
	/* Publish something on the topic */
    std_msgs::Float64 geo_msg; // init msg
	
	rosgraph_msgs::Clock clockMsg;

	clockMsg.clock = ros::Time(sim_t);
	time_publisher.publish(clockMsg);
	
	if (iteration%multiplier == 0 ) {
		Simulator_Step(false);
		
		ROS_INFO("Simulation executed, new Time: + %f = %f",sim_dt,sim_t);
	 	ROS_INFO("Simulator Step executed, theta = %f",sim_state[0]);
		
    	geo_msg.data = sim_state[0]; // loads data into it
		sim_publisher.publish(geo_msg);
	} else {
		
		// ROS_INFO("clock executed, new Time: + %f = %f",subtick,sim_t);
	}
	iteration++;
}

void node_sim::Simulator_Step(bool multi)
{
	
	ROS_INFO("Executing Simulator Step at t = %f", sim_t);
	stepper.do_step(std::bind(&node_sim::simulator_ode, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), sim_state, sim_t, sim_dt);

}

void node_sim::simulator_ode(const state_type &state, state_type &dstate, double t)
{
    // Actual state
    const double x1 = state[0]; // x1 = Omega
    const double x2 = state[1]; // x2 = Omega_dot

    // Model equations of a nonlinear pendulum with friction
    dstate[0] = x2;
    dstate[1] = -9.81/sim_l*std::sin(x1)-sim_d/(sim_m*std::pow(sim_l,2.0))*x2+sim_u/(sim_m*std::pow(sim_l,2.0));
}