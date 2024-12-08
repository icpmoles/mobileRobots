
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "node_example"

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
  
    
  public:
    double RunPeriod; 
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
    feedback_subscriber = Handle.subscribe("/tb_pose", 1, &node::tb_MessageCallback, this);
 	publisher = Handle.advertise<std_msgs::Float64MultiArray>("/lookahead_cmd", 1);

    theta = 0.0;
	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void node::RunPeriodically(float Period)
{	
	ros::Rate LoopRate(1.0/Period);

	ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
    ROS_INFO("Node %s: params xr: %f yr: %f R: %f T: %f", ros::this_node::getName().c_str(),xr,yr,R,T);  
	while (ros::ok()) {
		ros::spinOnce(); 
		LoopRate.sleep();
	}
}


void node::Shutdown(void) {
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}



void node::tb_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {

	theta = msg->data[3];
    double xp = msg->data[4];
    double yp = msg->data[5];
    
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
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE); 
  node node_node;
  node_node.Prepare();
  
  node_node.RunPeriodically(0.01);
   
  node_node.Shutdown();
  
  return (0);
}