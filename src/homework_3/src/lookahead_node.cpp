
#include "ros/ros.h"

#include "std_msgs/Float64MultiArray.h"
// i want to send float64 msgs between the example nodes
 #define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "node_example"

class node
{
  private: 
    ros::NodeHandle Handle; //ROS Handle
    // initialized at ros::init
    // needs to be used if you call a function of the 
    // ROS client library
    
    /* ROS topics */
    ros::Subscriber example_subscriber;
    ros::Subscriber feedback_subscriber;
    ros::Publisher publisher;
    
    /* Parameters from ROS parameter server */
    // param_type ParamVar;
    // where to store the parameters retrieved by the param server

    /* ROS topic callbacks */
    // void topic1_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void tb_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    /* Node periodic task */
    // void PeriodicTask(void);
    
    /* Node state variables */
    double xp_dot,yp_dot,theta,xr,yr;
    double R,T;
    std::string node_name;
  
    
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

void node::Prepare(void) // Janitor tasks
{
	 RunPeriod = RUN_PERIOD_DEFAULT;

	/* Retrieve parameters from ROS parameter server */
	// std::string FullParamName;

	// run_period
    node_name = ros::this_node::getName();
	// FullParamName = ros::this_node::getName()+"/run_period";

    Handle.getParam(node_name+"/xr", xr);
    Handle.getParam(node_name+"/yr", yr);
    	
	Handle.getParam(node_name+"/R",R);
	Handle.getParam(node_name+"/T",T);
	// Handle.getParam(FullParamName, RunPeriod)
	// FullParamName is a path
	// RunPeriod is where it stores the parameter


	/* ROS topics */
	// create sub/pub 
	// example_subscriber = Handle.subscribe("/lookahead_trajectory", 1, &node::topic1_MessageCallback, this);
    feedback_subscriber = Handle.subscribe("/tb_pose", 1, &node::tb_MessageCallback, this);
	// "/topic1",		topic name
	// 1,  				buffer size. 1 = as real time as possible.
	// &node::topic1_MessageCallback, 
	// function callback: what gets executed when a msg is received
	// this, 			pointer to the object of the class
	// when a callback is implemented in an object way
	// it needs to know the pointer to the node handle
 	 publisher = Handle.advertise<std_msgs::Float64MultiArray>("/lookahead_cmd", 1);

	// std_msgs::Float64,  type of msg we are advertising
	// "/topic2", 			topic name
	// 1: buffer size, like for subscriber

    theta = 0.0;
	/* Node variable initialization */
	// topic1_data = 0.0;  // default value if it hasn't been received already

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void node::RunPeriodically(float Period)
{	

	//
	ros::Rate LoopRate(1.0/Period);

	ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

    ROS_INFO("Node %s: params xr: %f yr: %f R: %f T: %f", ros::this_node::getName().c_str(),xr,yr,R,T);  
	// infinite cycle
	// ros::ok always true unless: 
	//ctrl+c from the terminal 
	//or call/receives a Ros:kill command
	while (ros::ok()) 
	{
		// PeriodicTask(); // tasks I actually do...
        // ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

		ros::spinOnce(); 
		// after you completed your little tasks,
		// execute eventual callbacks that you received in the meanwhile

		LoopRate.sleep();
		// sleep until the next time slot
	}
}


void node::Shutdown(void)
{
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
	// return 0 in the _core
	// put here janitorial functions for e.g. safe robot shutdown

}



// const std_msgs::Float64::ConstPtr& msg
// constant 
// & pointer
// to a message std_msgs::Float64 
// void node::topic1_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& recmsg)
// {
// 	/* Receive data from the topic */
// 	xp_dot = recmsg->data[1];
//     yp_dot = recmsg->data[2];

//     double v, omega;

//     v = (cos(theta)-yr*sin(theta)/xr)*xp_dot+(sin(theta)+yr*cos(theta)/xr)*yp_dot;
//     omega = (- sin(theta)*xp_dot + cos(theta)*yp_dot)/xr;


//     std_msgs::Float64MultiArray msg;
//     msg.data.resize(3);
//     msg.data[0] = ros::Time::now().toSec();
//     msg.data[1] = v;
//     msg.data[2] = omega;
//     publisher.publish(msg);


// }

void node::tb_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

   
	/* Receive data from the topic */
	theta = msg->data[3];


    double t = ros::Time::now().toSec();
    double phi = 2*3.14/T;
    xp_dot = - R * phi * sin(phi*t);
    yp_dot = R * phi * cos(phi*t);

    double v, omega;

    v = (cos(theta)-yr*sin(theta)/xr)*xp_dot+(sin(theta)+yr*cos(theta)/xr)*yp_dot;
    omega = (- sin(theta)*xp_dot + cos(theta)*yp_dot)/xr;


    std_msgs::Float64MultiArray msgtosend;
    msgtosend.data.resize(3);
    msgtosend.data[0] = ros::Time::now().toSec();
    msgtosend.data[1] = v;
    msgtosend.data[2] = omega;

    ROS_INFO("Node %s: received theta: %f\n calculated v:%f w: %f", ros::this_node::getName().c_str(),theta,v,omega);
    publisher.publish(msgtosend);
    // yp_dot = msg->data[2];
}

// void node_example::PeriodicTask(void)
// {
// 	/* Put here the code related to the node task */
// 	/* Publish something on the topic */
//     std_msgs::Float64MultiArray msg; // init msg
//     msg.data = topic1_data; // loads data into it
// 	publisher.publish(msg); // publish it.
// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE); // initialization of the node w/ ROS master
  // NAME_OF_THIS_NODE needs to be unique, can be passed by roslaunch or taken from the header file
  
  node node_node;
   // class containing all the functionalities of the node

  // Usual code structture of every controller in the world
  // setup: run once at start
  // loop : runs forever according to a: RunPeriod is a special variable that can be
  //                                     - default Period built in the header file
  //                                     - taken by a ROS param at initialization
  // shutdown: function to kill the controller and to exit

  node_node.Prepare();
  
  node_node.RunPeriodically(0.01);
   
  node_node.Shutdown();
  
  return (0);
}