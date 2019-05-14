/******************************************************************************************
*
*
* Setpoint node for the PD controller with gravity compensation and dynamic reconfiguration.
* This is intended to be a path planning node, or some sort of node that can
* send a sequence of joint configurations leading to a path.
* Path/trajectory stuff not implemented.
* Date sometime in July 2018
* If this code works, it was written by Eirik Kvalheim. If not, I don’t know who wrote it.
*
*
******************************************************************************************/


#include "ros/ros.h"
#include <ros/time.h>
#include <stddef.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>


// My Man!
int main(int argc, char **argv){

  // This is where it all begins...
  ros::init(argc, argv, "setpoint_node");
  ROS_INFO("Starting setpoint publisher");
  ros::NodeHandle setpoint_node;
  double uL_q = 1.0; // Upper limit joint variable
  double lL_q = 1.0; // Lower limit joint variable

  // Everybody loves information, especially from ROS
  while (ros::ok() && ros::Time(0) == ros::Time::now()){
    ROS_INFO("Setpoint_node spinning, waiting for time to become non-zero");
    sleep(1);
  }

  // Publish floating point setpoint messages every 0.1 seconds (10Hz)
  std_msgs::Float64 setpoint;
  setpoint.data = 0.0;
  ros::Publisher setpoint_pub = setpoint_node.advertise<std_msgs::Float64>("/pd_w_g/setpoint0", 1); // Choosing another topic here because using the rqt node to publish setpoints for now.
  ros::Rate loop_rate(10);

  // Publish setpoints until shut down
  while (ros::ok()){

    // Pumping callbacks
    ros::spinOnce();

    //https://www.youtube.com/watch?v=K91imfIWMV4
	  if (setpoint.data > 1.0) setpoint.data = 1.0;
	  if (setpoint.data < -1.0) setpoint.data = -1.0;
    setpoint_pub.publish(setpoint);
    loop_rate.sleep();

  }

  // We all love zeros
  return 0;

}
