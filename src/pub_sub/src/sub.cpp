#include "ros/ros.h"
#include "std_msgs/String.h"


void chatterCallback(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard: [%s]", msg->data.c_str()); //print out we receive a message
}

int main(int argc, char **argv){
  	// nb: no loop

	ros::init(argc, argv, "listener"); // initialize node with a name

	ros::NodeHandle n;
  	ros::Subscriber sub = n.subscribe("chatter", 1, chatterCallback);
	// create handle subscribe to topic "chatter" with a buffer = 1, it can be
	// increased if the elaboration takes too much time
	// starts the processing of the messahe through the chattercallback handle 

  	ros::spin(); // not spinOnce because we are not inside a loop 

  return 0;
}