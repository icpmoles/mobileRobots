#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
// #include "geometry_msgs/Pose.h"
// #include "geometry_msgs/Twist.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "tester"); // intialize the node with a meaningful name
	
	//, ros::init_options::AnonymousName); // intialize the node with an "automatic random generated" name without needing to remap at every start in case of conflict
	ros::NodeHandle n; //create nodeHandle

	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("cmd", 1);
	std::string node_name = ros::this_node::getName();
	float a,T,v,omega;
	// Handle.getParam(run_period_name, RunPeriod);

	
	n.getParam(node_name+"/a",a);
	n.getParam(node_name+"/T",T);

	//create publisher object with node.advertise with a type string and name "chatter" and 1 as size of the buffer of the publisher (1 is good most of the time)(can be increased in case your calculations take too much time)

	ros::Rate loop_rate(10.0);
	// running frequency of the loop at 10Hz

	int count = 0;

  	while (ros::ok()){ //standard ros loop, check if ROS is working, exit otherwise
			
			float t = ros::Time::now().toSec();
			// switches velocity back and forth
			// v = v_par * std::ceil(std::sin(3.14 * ros::Time::now().toSec() / (wave_period) ));
			float phi = 2*3.14/T;
			float y_d= a*phi*(cos(phi*t)*cos(phi*t)-sin(phi*t)*sin(phi*t)) ;
			float y_dd = -2* a *phi*phi* sin(phi*t)*cos(phi*t);
			float x_d = a*phi*cos(phi*t);
			float x_dd = -a *phi*phi*sin(phi*t);

			v = sqrt(x_d*x_d+y_d*y_d);
			omega = (x_d*y_dd-y_d*x_dd)/(x_d*x_d+y_d*y_d);
	
			// omega = - omega_par * (0.5- std::ceil(-std::sin(3.14 * ros::Time::now().toSec() / (wave_period) )));

			

	    	std_msgs::Float64MultiArray msg;
			msg.data.resize(3);
			msg.data[0] = ros::Time::now().toSec();
			msg.data[1] = v;
			msg.data[2] = omega;

			// std_msgs::Float64MultiArray dim[3] ;
			// dim[0].label  = "height"
			//  dim[0].size   = 480
			//  dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
			//  dim[1].label  = "width"
			//  dim[1].size   = 640
			//  dim[1].stride = 3*640 = 1920
			//  dim[2].label  = "channel"
			//  dim[2].size   = 3
			//  dim[2].stride = 3
			

        
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