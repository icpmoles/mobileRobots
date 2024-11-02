#include "node_simulator.h"
// #include "simulator_odefun/simulator_odefun.h"
// #include "simulator_odefun/simulator_odefun.h"

#define DT 0.001
#define M 10.0
#define L 1.0
#define D 4.0
#define X10 0.0
#define X20 0.0
#define NUM_STEP 10000



void node_sim::Prepare(void) // Janitor tasks
{
	// float m;
	// float l;
	// float d;

	/* Retrieve parameters from ROS parameter server */
	std::string m_name,l_name,d_name,dt_name,run_period_name;

	// run_period
	
	run_period_name = ros::this_node::getName()+"/run_period";
	m_name = ros::this_node::getName()+"/m";
	
	d_name = ros::this_node::getName()+"/d";
	
	l_name = ros::this_node::getName()+"/l";
	dt_name = ros::this_node::getName()+"/dt";
	Handle.getParam(m_name, sim_m);
	Handle.getParam(l_name, sim_l);
	Handle.getParam(d_name, sim_d);
	
	// Handle.getParam(dt_name, sim_dt);
	Handle.getParam("/tick_multiplier",multiplier);
	Handle.getParam("/subtick",subtick);
	sim_dt = subtick*multiplier;
	// Handle.getParam(FullParamName, RunPeriod)
	// FullParamName is a path
	// RunPeriod is where it stores the parameter

	if (true == Handle.getParam(run_period_name, subtick))
	{
		ROS_INFO("Node %s: retrieved parameter %s.",
				ros::this_node::getName().c_str(), run_period_name.c_str());

		// FullParamName.c_str() = some functions prefer a "C style" string
		// Some functions prefer it this way
	}
	else
	{
		ROS_ERROR("Node %s: unable to retrieve parameter %s.",
				ros::this_node::getName().c_str(), run_period_name.c_str());
	}

	/* ROS topics */
	// create sub/pub 
	sim_subscriber = Handle.subscribe("/controller_cmd", 3, &node_sim::sub_callback, this);
	// "/topic1",		topic name
	// 1,  				buffer size. 1 = as real time as possible.
	// &node_sim::topic1_MessageCallback, 
	// function callback: what gets executed when a msg is received
	// this, 			pointer to the object of the class
	// when a callback is implemented in an object way
	// it needs to know the pointer to the node handle
 	sim_publisher = Handle.advertise<std_msgs::Float64>("/simulation_output", 5);
	time_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 10);
	// std_msgs::Float64,  type of msg we are advertising
	// "/topic2", 			topic name
	// 1: buffer size, like for subscriber
	
    // simulator_odefun my_simulator(dt);
    // my_simulator.setInitialState(X10, X20);
    // my_simulator.setModelParams(m, l, d);
	sim_state.resize(2); // specify size of state
    sim_state[0] = 0.0;
    sim_state[1] = 0.0;	

	setInitialState(X10,X20);
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

	
	// Simulator_Step(true);
	// ROS_INFO("Sim: end of first simulation");
	// ros::spinOnce(); 
	
	// ROS_INFO("Sim: spinOnce");
	// sleep(0.2);

	
	// ROS_INFO("Sim: spinOnce again");
	// infinite cycle
	// ros::ok always true unless: 
	//ctrl+c from the terminal 
	//or call/receives a Ros:kill command
	while (ros::ok()) 
	{
		
		// ROS_INFO("Sim: ros ok loop?");
		// PeriodicTask(); // tasks I actually do...
		ros::spinOnce(); 
		PeriodicTask(); 
		// after you completed your little tasks,
		// execute eventual callbacks that you received in the meanwhile
		
		// ROS_INFO("Simulator: new spin %f Hz",1/Period);
		LoopRate.sleep();
		// usleep(100000);
		// sleep until the next time slot
	}
}


void node_sim::Shutdown(void)
{
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
	// return 0 in the _core
	// put here janitorial functions for e.g. safe robot shutdown

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
	
	
	// ROS_INFO("Simulator: periodic TASK");

	
	 // Update time
    sim_t += (subtick);
	// starts simulations
	/* Put here the code related to the node task */
	/* Publish something on the topic */
    std_msgs::Float64 geo_msg; // init msg
    geo_msg.data = sim_state[0]; // loads data into it
	 // publish it.

	// time pub
	rosgraph_msgs::Clock clockMsg;
	// what's the new time after running the simulation?

	clockMsg.clock = ros::Time(sim_t);
	time_publisher.publish(clockMsg);
	
	// usleep(10);
	/* Put here the code related to the node task */
	/* Publish something on the topic */
    // std_msgs::Float64 msg; // init msg
    // msg.x = sim_y1; // loads data into it
	// msg.y = sim_y2; // loads data into it
	// sim_publisher.publish(msg); // publish it.

	// // time pub
	// rosgraph_msgs::Clock clockMsg;
	// // what's the new time after running the simulation?

	// clockMsg.clock = ros::Time(sim_t);
	// time_publisher.publish(clockMsg);
	if (iteration%multiplier == 0 ) {
		Simulator_Step(false);
		
		ROS_INFO("Simulation executed, new Time: + %f = %f",sim_dt,sim_t);
	 	ROS_INFO("Simulator Step executed, theta = %f",sim_state[0]);
		sim_publisher.publish(geo_msg);
	} else {
		
		ROS_INFO("clock executed, new Time: + %f = %f",subtick,sim_t);
	}
	iteration++;
}

void node_sim::Simulator_Step(bool multi)
{
	
	ROS_INFO("Executing Simulator Step at t = %f", sim_t);
	stepper.do_step(std::bind(&node_sim::simulator_ode, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), sim_state, sim_t, sim_dt);

    // Update time
    // sim_t += sim_dt;
	// starts simulations
	/* Put here the code related to the node task */
	/* Publish something on the topic */
    // std_msgs::Float64 geo_msg; // init msg
    // geo_msg.data = sim_state[0]; // loads data into it
	// geo_msg.y = sim_y2; // loads data into it
	 // publish it.

	// time pub
	// rosgraph_msgs::Clock clockMsg;
	// what's the new time after running the simulation?

	// clockMsg.clock = ros::Time(sim_t);
	// time_publisher.publish(clockMsg);
	// sim_publisher.publish(geo_msg);
	
	
	// ROS_INFO("Simulator Step executed, new Time: + %f = %f",sim_dt,sim_t);
	// ROS_INFO("Simulator Step executed, theta = %f",sim_state[0]);
}

void node_sim::simulator_ode(const state_type &state, state_type &dstate, double t)
{
    // Actual state
    const double x1 = state[0];
    const double x2 = state[1];

    // Model equations of a nonlinear pendulum with friction
    dstate[0] = x2;
    dstate[1] = -9.81/sim_l*std::sin(x1)-sim_d/(sim_m*std::pow(sim_l,2.0))*x2+sim_u/(sim_m*std::pow(sim_l,2.0));
}