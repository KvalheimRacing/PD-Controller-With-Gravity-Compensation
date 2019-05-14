/******************************************************************************************
*
*
* PD controller with  gravity compensation and dynamic reconfigure
* Date sometime in July 2018
* If this code works, it was written by Eirik Kvalheim. If not, I don’t know who wrote it.
*
*
******************************************************************************************/


#ifndef PD_CONTROLLER_W_GRAVITY
#define PD_CONTROLLER_W_GRAVITY

#include "math.h"
#include "ros/ros.h"
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <pd_w_g/pd_w_gConfig.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <string>
#include <eigen3/Eigen/Eigen>


namespace Eigen { typedef Matrix<double,6,1> Vector6d; }              // Defining custom eigen vector
namespace crustcrawler{                                               // Using the same namespace as the rest of the robot


  // Class for the PD controller with g.. you know how it goes
  class PD_w_g_Object{

      public:

          // Nothing to write here
          PD_w_g_Object(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

          // Primary vectors
          Eigen::Vector6d tau;                                        // Control effort from PD w gravity controller
          Eigen::Vector6d q_d;                                        // Desired joint variables
      		Eigen::Vector6d q;                                          // Joint variables
      		Eigen::Vector6d qd;                                         // Joint velocities (qd)
          Eigen::Vector6d qd_tmp;                                     // Vector for temporary joint velocities
          Eigen::Vector6d K_P;                                        // Vector with the proportional gain
          Eigen::Vector6d K_D;                                        // Vector with the derivative gain
          Eigen::Vector6d Slope;                                      // Rate of change between the current and previous effort
          Eigen::MatrixXd pv_deriv_hist;                              // Matrix with the velocity history (derivatives of the process value)
          Eigen::MatrixXd control_effort_hist;                        // Matrix with the control effort history
          Eigen::MatrixXd filtered_pv_deriv_hist;                     // Filtered data of velocity
          Eigen::MatrixXd filtered_control_effort_hist;               // Filtered data of control effort


      private:

          // Meh
          void getParameters(double in, double& value, double& scale);
          void pd_w_g_EnableCallback(const std_msgs::Bool& pd_w_g_enable_msg);
          void reconfigureCallback(crustcrawler_controllers::pd_w_gConfig& config, uint32_t level);
          void callbackJointState(const sensor_msgs::JointState& joint_state);
          void assignDataToPublish(Eigen::Vector6d joint_torque);
          void printData();
          bool validateParameters();
          void setpointCallback(const std_msgs::Float64& setpoint_msg);
          void lowPassFilter();
          void getJointStates(const sensor_msgs::JointState& joint_state);
          void cutoffCallback(const std_msgs::Float64& cutoff_frequency_velocity_msg);
          void cutoffVelocityCallback(const std_msgs::Float64& cutoff_frequency_velocity_msg);
          void cutoffEffortCallback(const std_msgs::Float64& cutoff_frequency_effort_msg);
          void newSetpointCallback(const sensor_msgs::JointState& joint_state);

          // Input variables for controller
          bool pd_w_g_enabled = true;                                 // Controller is enabled to run
          bool new_const_setpoint = false;                            // Indicate that a new setpoint (same for all joints) is given
          bool first_reconfig = true;                                 // Used to set initial Kp and Kd values
          bool simulation_toggle = false;                             // Used to specify whether the node is launched in simulation or on real robot
          int simCounter = 2;                                         // If simulation is activated, joint state messages start on i+2

          // Variables for low pass filter applied on the velocity/derivative calculation on joint 6
          ros::Time prev_time;
          ros::Duration delta_t;
          double joint_6_velocity = 0;                                // Storing the velocity of joint 6
          double cutoff_frequency_velocity = 0.001;                   // The cutoff frequency of the low-pass filter on the derivative term (in Hz) 0.001 as default
          double cutoff_frequency_effort = 100;                       // The cutoff frequency of the low-pass filter on the effort (in Hz) 100 as default
          double c_effort = 3.0;                                      // Default 3.0 corresponds to a cutoff frequency at 3/4 of the sample rate
          double c = 1000.0;                                          // Default 1000.0 corresponds to a cutoff frequency at 250 times the sample rate
          double tan_filt = 1.;                                       // Used to check for tan(0)==>NaN in the filter calculation

          // PD gains
          double Kp_ = 0,  Kd_ = 0;

          // Upper and lower saturation limits for control effort
          double uL = 5, lL = -5;

          // Defining publisher and subscriber instances
          ros::Publisher pub_j1, pub_j2, pub_j3, pub_j4, pub_j5, pub_j6;
          ros::Subscriber sub_joint_states, sub_setpoint, sub_pd_w_g_enabled, sub_cutoff_frequency_velocity, sub_cutoff_frequency_effort, sub_new_setpoint;

          std::string topic1_from_controller, topic2_from_controller, topic3_from_controller, topic4_from_controller, topic5_from_controller, topic6_from_controller;
          std::string topic_from_crustcrawler, setpoint_topic, enable_topic, cutoff_frequency_velocity_topic,cutoff_frequency_effort_topic, new_setpoint_topic;

  };      // end class

}         // end crustcrawler namespace

#endif
