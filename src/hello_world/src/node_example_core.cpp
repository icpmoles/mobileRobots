#include "hello_world/node_example.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  node_example node_example_node;
   
  node_example_node.Prepare();
  
  node_example_node.RunPeriodically(node_example_node.RunPeriod);
   
  node_example_node.Shutdown();
  
  return (0);
}

