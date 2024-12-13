
#include "ros/ros.h"

#include "std_msgs/Float64MultiArray.h"
#define RUN_PERIOD_DEFAULT 0.1
 
#define NAME_OF_THIS_NODE "node_example"

double step( double t, double ts){
  if (t<ts)
    return 0.0;
  else 
    return 1.0;
}

class node
{
  private: 
    ros::NodeHandle Handle; 
    ros::Subscriber example_subscriber;
    ros::Subscriber feedback_subscriber;
    ros::Publisher publisher;
    void tb_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    double xp_dot,yp_dot,theta,xr,yr;
    double R,T;
    std::string node_name;
    double ta,tb, a, b;

   
    
  public:
    double RunPeriod; 
    
    double refreshperiod;
    void Prepare(void);
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
  Handle.getParam(node_name+"/a",a);
  Handle.getParam(node_name+"/b",b);
  Handle.getParam(node_name+"/ta",ta);
  Handle.getParam(node_name+"/tb",tb);
  feedback_subscriber = Handle.subscribe("/state", 1, &node::tb_MessageCallback, this);
 	publisher = Handle.advertise<std_msgs::Float64MultiArray>("/lookahead_cmd", 1);
  theta = 0.0;
	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}



void node::RunPeriodically(float Period)
{	
	ros::Rate LoopRate(1.0/Period);
	ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
  ROS_INFO("Node %s: params xr: %f yr: %f R: %f T: %f", ros::this_node::getName().c_str(),xr,yr,R,T);  
	while (ros::ok()) 
	{
		ros::spinOnce(); 
		LoopRate.sleep();
	}
}


void node::Shutdown(void)
{
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());

}



void node::tb_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

	theta = msg->data[3];

  double t = ros::Time::now().toSec();
  double phi = 2*3.14/T;

  xp_dot = a *  step (t,ta);
  yp_dot = b * step (t,tb);

  double v, omega;

  v = (cos(theta)-yr*sin(theta)/xr)*xp_dot + (sin(theta)+yr*cos(theta)/xr)*yp_dot;
  omega = (- sin(theta)*xp_dot + cos(theta)*yp_dot)/xr;

  std_msgs::Float64MultiArray msgtosend;
  msgtosend.data.resize(3);
  msgtosend.data[0] = ros::Time::now().toSec();
  msgtosend.data[1] = v;
  msgtosend.data[2] = omega;
  publisher.publish(msgtosend);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  node node_node();
  node_node.Prepare();
  node_node.RunPeriodically(node_node.refreshperiod);
  node_node.Shutdown();
  return (0);
}