#include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "tester"); // intialize the node with a meaningful name
	
	//, ros::init_options::AnonymousName); // intialize the node with an "automatic random generated" name without needing to remap at every start in case of conflict
	ros::NodeHandle n; //create nodeHandle

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd", 1);
	std::string node_name = ros::this_node::getName();
	float v,omega,scan_period,v_par,omega_par,wave_period,c_t;
	// Handle.getParam(run_period_name, RunPeriod);

	n.getParam(node_name+"/v",v_par);
	n.getParam(node_name+"/omega",omega_par);
	n.getParam(node_name+"/scan_period",scan_period);
	n.getParam(node_name+"/wave_period",wave_period);

	//create publisher object with node.advertise with a type string and name "chatter" and 1 as size of the buffer of the publisher (1 is good most of the time)(can be increased in case your calculations take too much time)

	ros::Rate loop_rate(1.0/scan_period);
	// running frequency of the loop at 10Hz

	int count = 0;

  	while (ros::ok()){ //standard ros loop, check if ROS is working, exit otherwise
			
			c_t = fmod(ros::Time::now().toSec() ,wave_period);
			// switches velocity back and forth
			// v = v_par * std::ceil(std::sin(3.14 * ros::Time::now().toSec() / (wave_period) ));

			v= v_par;
			// omega = - omega_par * (0.5- std::ceil(-std::sin(3.14 * ros::Time::now().toSec() / (wave_period) )));

			if (c_t>10.0 && c_t<14.0){
				omega = omega_par;
			} else {
				omega = 0;
			}

	    	geometry_msgs::Twist msg; 

                
			msg.linear.x = v; //ros msg object only has data field
			msg.angular.z = omega;
    		// ROS_INFO("%s: Publishing test vel_cmd for turtlebot: %f %f",node_name.c_str(),v,omega); //standard ROS logging/debugging message
			// we just log the content of our msg

    		chatter_pub.publish(msg);
			// we just publish the content of our msg

    		ros::spinOnce();
			// handles all the other generic ros tasks like callbacks timers etc, useful for more complex
			// code

    		loop_rate.sleep(); //just waits until the next 10Hz loop, better this way 
			// instead of the built in sleep
			

			
  	}


  	return 0;
}