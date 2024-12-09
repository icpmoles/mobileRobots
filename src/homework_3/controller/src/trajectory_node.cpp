#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "loop_traj"); 
	ros::NodeHandle n; 

	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("lookahead_trajectory", 1);
	std::string node_name = ros::this_node::getName();
	double R,T,xp_dot,yp_dot;
	n.getParam(node_name+"/R",R);
	n.getParam(node_name+"/T",T);
	ros::Rate loop_rate(10.0);
	// ROS_INFO("%s: R: %f  T:%f", ros::this_node::getName().c_str(),R,T);
	int count = 0;
  	while (ros::ok()){ 
			double t = ros::Time::now().toSec();
			double phi = 2*3.14/T;
            xp_dot = - R * phi * sin(phi*t);
			yp_dot = R * phi * cos(phi*t);
	    	std_msgs::Float64MultiArray msg;
			msg.data.resize(3);
			msg.data[0] = ros::Time::now().toSec();
			msg.data[1] = xp_dot;
			msg.data[2] = yp_dot;
    		chatter_pub.publish(msg);
			// ROS_INFO("%s: xp_dot: %f  yp_dot:%f", ros::this_node::getName().c_str(),xp_dot,yp_dot);
    		ros::spinOnce();
    		loop_rate.sleep(); 
  	}

  	return 0;
}