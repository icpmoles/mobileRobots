#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sstream>


int m;
int c;
int ci;
int cf;

void subCallback(const std_msgs::Int32::ConstPtr& msg){
//   ROS_INFO("I heard: [%s]", msg->data.c_str()); //print out we receive a message
  m = msg->data;
  if (m<cf){
    c=m;
  }
}


int main(int argc, char **argv){
    
	ros::init(argc, argv, "param_first");
	ros::NodeHandle n; // this is a global node handle
	ros::NodeHandle nh_private("~"); // this is a private node handle



	ros::Publisher pub = n.advertise<std_msgs::Int32>("counter", 1000); // publish global topic, no node name

    ros::Subscriber sub = n.subscribe("set_counter", 1, subCallback);

	
	std::string name;
	n.getParam("name", name);  //get global param
	
	std::string local_name;
	nh_private.getParam("cf", cf); //get local param
    
	nh_private.getParam("ci", ci); //get local param
    c = ci;
	
	std::string local_name_from_global;
	std::string param_name = ros::this_node::getName() + "/name"; // we build the "path" of the parameter, it works somehow
	// ROS_INFO("local param name: %s", param_name.c_str()); // we use the ROS api to retrieve the parameter by providing the path, this can be done local to local but also from other nodes
	n.getParam(param_name, local_name_from_global);  //get local param using global nodehandle

	ros::Rate loop_rate(1);

  
  	while (ros::ok()){
           ROS_INFO("Counter: %i, ci %i; cf %i",  c,ci,cf);
           


	    	std_msgs::Int32 msg;
    		msg.data = c;
    		

            if (c==cf)
    		{pub.publish(msg);}

            if (c<=cf){ c++;}
    		ros::spinOnce();

    		loop_rate.sleep();
    		
  	}


  	return 0;
}