#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "loop_traj"); 
	ros::NodeHandle n; 

	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("tb_cmd", 1);
	std::string node_name = ros::this_node::getName();
	float R,T,v,omega;
	n.getParam(node_name+"/R",R);
	n.getParam(node_name+"/T",T);
	ros::Rate loop_rate(10.0);

	int count = 0;
  	while (ros::ok()){ 
			float t = ros::Time::now().toSec();
			float phi = 2*3.14/T;
			v = R * phi;
			omega = phi ; 
	    	std_msgs::Float64MultiArray msg;
			msg.data.resize(3);
			msg.data[0] = ros::Time::now().toSec();
			msg.data[1] = v;
			msg.data[2] = omega;
    		chatter_pub.publish(msg);
    		ros::spinOnce();
    		loop_rate.sleep(); 
  	}

  	return 0;
}