#include "hw2_simulator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE); 
  node_sim node;

  node.Prepare();
  
  node.RunPeriodically(node.subtick);
   
  node.Shutdown();
  
  return (0);
}

