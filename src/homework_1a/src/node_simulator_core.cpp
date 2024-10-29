// it contains the main...., just copy paste and modify if needed
#include "node_simulator.h"
// #include "simulator_odefun/simulator_odefun.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE); // initialization of the node w/ ROS master
  // NAME_OF_THIS_NODE needs to be unique, can be passed by roslaunch or taken from the header file
  
  node_sim node;
   // class containing all the functionalities of the node

  // Usual code structture of every controller in the world
  // setup: run once at start
  // loop : runs forever according to a: RunPeriod is a special variable that can be
  //                                     - default Period built in the header file
  //                                     - taken by a ROS param at initialization
  // shutdown: function to kill the controller and to exit

  node.Prepare();
  
  node.RunPeriodically(node.RunPeriod);
   
  node.Shutdown();
  
  return (0);
}

