
#include "ros/ros.h"
// #include "hw4_PID.h"
#include "std_msgs/Float64MultiArray.h"
#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "node_example"


class node
{
  private: 
    ros::NodeHandle Handle; 
    ros::Subscriber example_subscriber;
    ros::Subscriber feedback_subscriber;
    ros::Publisher publisher,tr_publisher;
    
    void tb_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    double xp_dot,yp_dot; // velocity of trajectory: feedforward
    double xp_,yp_;       // cartesian coordinates of trajectory: Feed into PID


    double theta,x,y,yp,xp; //states of the robot
    double xr,yr;     // feedback linearization parameters
    double R,T;       // trajectory parameters for circle/eight
    std::string node_name;
  
    double pid_kc; //, pid_ti;      // PID parameters
    // PID PIDx, PIDy; // le PID: initialized with the same parameter

  public:
    double refreshperiod;
    double RunPeriod; 
    void Prepare(void);
    void PeriodicTask(void);
    void RunPeriodically(float Period);
    void Shutdown(void);
};

void node::Prepare(void)
{
	 RunPeriod = RUN_PERIOD_DEFAULT;

  node_name = ros::this_node::getName();

  Handle.getParam("/xr", xr);
  Handle.getParam("/yr", yr);
    	
	Handle.getParam(node_name+"/R",R);
	Handle.getParam(node_name+"/T",T);
  Handle.getParam(node_name+"/refreshperiod",refreshperiod);
  Handle.getParam(node_name+"/pidKc",pid_kc);
  feedback_subscriber = Handle.subscribe("/state", 1, &node::tb_MessageCallback, this);
 	publisher = Handle.advertise<std_msgs::Float64MultiArray>("/cmd", 1);
  tr_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/trajectory", 1);

  theta = 0.0;
	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void node::RunPeriodically(float Period)
{	
	ros::Rate LoopRate(1.0/Period);

	ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
  ROS_INFO("Node %s: params xr: %f yr: %f R: %f T: %f", ros::this_node::getName().c_str(),xr,yr,R,T);  

	while (ros::ok()) {
    node::PeriodicTask();
		ros::spinOnce(); 
		LoopRate.sleep();
	}
}


void node::Shutdown(void) {
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void node::PeriodicTask(void) {

	double t = ros::Time::now().toSec();
  double phi = 2*3.14/T;
  // TRAJECTORY GENERATION  
  xp_dot = - R * phi * sin(phi*t);
  yp_dot = R * phi * cos(phi*t);
  xp_ =  R * ( cos(phi*t) - 1);
  yp_ = R * sin(phi*t);


  // XP YP estimation

  xp = x + xr * cos(theta) - yr * sin(theta);
  yp = y + xr * sin(theta) + yr * cos(theta);

  // error computation
  double ex = xp_ - xp;
  double ey = yp_ - yp;

  // FEED FORWARD

  double Kp = pid_kc; 
  double vx = Kp * ex + xp_dot; 
  double vy = Kp * ey + yp_dot;

  // FEEDBACK LINEARIZATION
  double v, omega;
  v = (cos(theta)-yr*sin(theta)/xr)*vx+(sin(theta)+yr*cos(theta)/xr)*vy;
  omega = (- sin(theta)*vx + cos(theta)*vy)/xr;

  // PUBLISHING CMD
  std_msgs::Float64MultiArray msgtosend;
  msgtosend.data.resize(3);
  msgtosend.data[0] = t;
  msgtosend.data[1] = v;
  msgtosend.data[2] = omega;
  publisher.publish(msgtosend);

  // PUBLISHING TRAJECTORY
  std_msgs::Float64MultiArray msgtosend_tr;
  msgtosend_tr.data.resize(5);
  msgtosend_tr.data[0] = t;
  msgtosend_tr.data[1] = xp_;
  msgtosend_tr.data[2] = yp_;
  msgtosend_tr.data[3] = xp;
  msgtosend_tr.data[4] = yp;
  tr_publisher.publish(msgtosend_tr);
}

void node::tb_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    // data aquisition
    x   = msg->data[1];  
    y   = msg->data[2];   
	  theta = msg->data[3];
    
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE); 
  node node_node;
  node_node.Prepare();
  
  node_node.RunPeriodically(node_node.refreshperiod);
   
  node_node.Shutdown();
  
  return (0);
}