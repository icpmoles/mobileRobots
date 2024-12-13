#ifndef TEST_SIMPLEH_
#define TEST_SIMPLE_H_

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

#define RUN_PERIOD_DEFAULT 0.01
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */

#define NAME_OF_THIS_NODE "test_simple"


class test_simple
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Publisher vehicleCommand_publisher;

    /* Parameters from ROS parameter server */
    double V0, V1, omega0, omega1, T01;

    /* Node periodic task */
    void PeriodicTask(void);

    /* Node state variables */
    double xref, dxref, ddxref, yref, dyref, ddyref;
    double linear_velocity, angular_velocity;

  public:
    double RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

};

#endif /* TEST_SIMPLE_H_ */
