
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "node_example"
#include "hw4_PID.h"

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
  
    //pid

    double pid_b, pid_a ;
    double pid_kc, pid_ti;
    double pidx_u_act   = 0.0;
    double pidx_uI_prev = 0.0;
    double pidx_y_act   = 0.0;
    double pidx_ysp_act = 0.0;

    double pidy_u_act   = 0.0;
    double pidy_uI_prev = 0.0;
    double pidy_y_act   = 0.0;
    double pidy_ysp_act = 0.0;

    PID* PIDx, PIDy; //

  public:
    node(void);
    double refreshperiod;
    double RunPeriod; 
    void Prepare(void);
    void RunPeriodically(float Period);
    void Shutdown(void);
};

node::node(void) {
  PIDx = new PID(1.0,1.0,1.0);
  PIDy = new PID(1.0,1.0,1.0);
}

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
  Handle.getParam(node_name+"/pidTi",pid_ti);
  feedback_subscriber = Handle.subscribe("/state", 1, &node::tb_MessageCallback, this);
 	publisher = Handle.advertise<std_msgs::Float64MultiArray>("/lookahead_cmd", 1);

    theta = 0.0;

    double pid_ts = refreshperiod;
    pid_a = pid_kc*pid_ts/pid_kc;
    pid_b = pid_kc;
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
    // data aquisition
	  theta = msg->data[3];
    double xp = msg->data[4];
    double yp = msg->data[5];
    
    double t = ros::Time::now().toSec();
    double phi = 2*3.14/T;

    // xp_dot = derivative for feedroward
    xp_dot = - R * phi * sin(phi*t);
    yp_dot = R * phi * cos(phi*t);
    // xsp_dot = trajectory
    double xsp =  R * ( cos(phi*t) - 1);
    double ysp = R * sin(phi*t);

    // PID
        pidx_ysp_act = xsp;
        pidy_ysp_act = ysp;

        pidx_y_act = xp;
        pidy_y_act = yp;
        double ex = (pidx_ysp_act-pidx_y_act);
        double ey = (pidy_ysp_act-pidy_y_act);

        double pidx_uI_act = pidx_uI_prev+pid_a*(pidx_ysp_act-pidx_y_act);
        double pidx_uP_act = pid_b*(pidx_ysp_act-pidx_y_act);
        pidx_u_act = pidx_uP_act+pidx_uI_act+xp_dot;
        pidx_uI_prev = pidx_uI_act;

        double pidy_uI_act = pidy_uI_prev+pid_a*(pidy_ysp_act-pidy_y_act);
        double pidy_uP_act = pid_b*(pidy_ysp_act-pidy_y_act);
        pidy_u_act = pidy_uP_act+pidy_uI_act+yp_dot; //proportional + integrator + feedforward
        pidy_uI_prev = pidy_uI_act;    

    // linearization

    double v, omega;

    v = (cos(theta)-yr*sin(theta)/xr)*pidx_u_act+(sin(theta)+yr*cos(theta)/xr)*pidy_u_act;
    omega = (- sin(theta)*pidx_u_act + cos(theta)*pidy_u_act)/xr;

    // publishing
    std_msgs::Float64MultiArray msgtosend;
    msgtosend.data.resize(3);
    msgtosend.data[0] = ros::Time::now().toSec();
    msgtosend.data[1] = v;
    msgtosend.data[2] = omega;

    // ROS_INFO("Node %s: received theta: %f\n calculated v:%f w: %f", ros::this_node::getName().c_str(),theta,v,omega);
    publisher.publish(msgtosend);
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