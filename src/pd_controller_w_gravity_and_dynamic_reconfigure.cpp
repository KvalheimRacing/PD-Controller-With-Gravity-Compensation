/******************************************************************************************
*
*
* PD controller with  gravity compensation and dynamic reconfigure
* Date sometime in July 2018
* If this code works, it was written by Eirik Kvalheim. If not, I don’t know who wrote it.
*
*
******************************************************************************************/


#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <pd_w_g/pd_controller_w_gravity_and_dynamic_reconfigure.h>


// Using the same namespace as the rest of the robot
using namespace crustcrawler;


// Function for getting the 6x1 vector for the gravitational forces - calculated with measured link weights (including motors)
Eigen::Vector6d getGravity(Eigen::Vector6d q) {

		// Vector object
		Eigen::Vector6d gravity_vector;

		// Joint variables, T2 = Theta2 etc
		double T2 = q(1);
		double T3 = q(2);
		double T4 = q(3);
		double T5 = q(4);

		// Tmp variables
	  double t2 = std::sin(T2);
	  double t3 = std::cos(T2);
	  double t4 = std::sin(T3);
	  double t5 = std::cos(T3);
	  double t6 = std::cos(T4);
	  double t7 = std::sin(T5);
	  double t8 = std::cos(T5);
	  double t9 = t2 * t4 * t7 * 0.3731890352476; // Just guessing on some constant here really. Took some time though

		// Gravity vector, we see that 0.0 makes sence for gravity_vector(0) and gravity_vector(5) when the joint state of Theta1 and Theta6 cleary does not change the gravitational forces applied on the robot.
	  gravity_vector[0] = 0.0;
	  gravity_vector[1] = (((((((t2 * -2.0931098669844 + t9) - t2 * t5 * 1.1149865389773) - t3 *
	               					t4 * 1.1149865389773) - t2 * t5 * t6 * 0.45246033351035) - t3 *
	             						t4 * t6 * 0.45246033351035) - t3 * t5 * t7 * 0.3731890352476) - t2 *
	           							t5 * t6 * t8 * 0.3731890352476) - t3 * t4 * t6 * t8 * 0.3731890352476;
	  gravity_vector[2] = ((((((t9 - t2 * t5 * 1.1149865389773) - t3 * t4 * 1.1149865389773) -
	              					t2 * t5 * t6 * 0.45246033351035) - t3 * t4 * t6 * 0.45246033351035)
	            					- t3 * t5 * t7 * 0.3731890352476) - t2 * t5 * t6 * t8 *
	           							0.3731890352476) - t3 * t4 * t6 * t8 * 0.3731890352476;
	  gravity_vector[3] = std::cos(T2 + T3) * std::sin(T4) * (t8 * 760136.0 + 921601.0) * -4.9095035E-7;
	  gravity_vector[4] = ((t2 * t5 * t8 * -0.3731890352476 - t3 * t4 * t8 * 0.3731890352476) +
	           							t2 * t4 * t6 * t7 * 0.3731890352476) - t3 * t5 * t6 * t7 * 0.3731890352476;
	  gravity_vector[5] = 0.0;

		// returning computed vector
		return gravity_vector;

}




// Allow the enabling of the controller to be controlled by a topic. This is modifiable in case there are multiple controllers, which there often are for a real robot.
void PD_w_g_Object::pd_w_g_EnableCallback(const std_msgs::Bool& pd_w_g_enable_msg){

	  pd_w_g_enabled = pd_w_g_enable_msg.data;

		// Leaving already?
		return;

}




// Allow the cutoff frequency of the velocity to be controlled by a topic. This is pretty good now, but is modifiable in case one searches for ultimate performance
void PD_w_g_Object::cutoffVelocityCallback(const std_msgs::Float64& cutoff_frequency_velocity_msg){

	  cutoff_frequency_velocity = cutoff_frequency_velocity_msg.data;

		// Bye bye
		return;

}




// Allow the cutoff frequency of the effort to be controlled by a topic. (This is pretty good now,) but is modifiable in case one searches for ultimate performance
void PD_w_g_Object::cutoffEffortCallback(const std_msgs::Float64& cutoff_frequency_effort_msg){

	  cutoff_frequency_effort = cutoff_frequency_effort_msg.data;

		// Adios
		return;

}




// Allow the possibility of setting joint values withouth starting and stopping the controller node
void PD_w_g_Object::setpointCallback(const std_msgs::Float64& setpoint_msg){


		// This sets the setpoint equally on all joints
	  q_d.setConstant(setpoint_msg.data);
		new_const_setpoint = true;														// Currently only used to toggle subscribed setpoint topics, in the future one could use this or a similar variable for limiting calculations if there are no changes to the system, and the error is within a certain limit

		// To infinity and beyond
		return;

}




// Allow the possibility of setting joint values withouth starting and stopping the controller node
void PD_w_g_Object::newSetpointCallback(const sensor_msgs::JointState& joint_state){

		// Set individual values on all joints
		for(int i=0; i<6; i++) q_d(i) = joint_state.position[i];

		// Used to toggle subscribed setpoint topics
		new_const_setpoint = false;

		// L8r
		return;

}




// 6'th and 5'th Joint is moving like crazy, so applying LPF here. Reference https://ccrma.stanford.edu/~jos/filters/
void PD_w_g_Object::lowPassFilter() {

		// Calculate delta_t
		if (!prev_time.isZero()){  // Not first time through the loop

			// I think we can all agree that this does not need commenting
			delta_t = ros::Time::now() - prev_time;
			prev_time = ros::Time::now();

			// Failsafe
			if (0 == delta_t.toSec()){

				ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f", ros::Time::now().toSec());

				// Escape madness
				return;

			}

		}
		else {	// prev_time is 0

			// Set value and return
			prev_time = ros::Time::now();

			// Got what we wanted
			return;

		}

		// Calculate new c for velocity if we have a user specified cutoff frequency for velocity
		if (cutoff_frequency_velocity != 0.001){

				// Check if tan(_) is really small, could cause c = NaN
				tan_filt = tan((cutoff_frequency_velocity * 6.2832) * delta_t.toSec() / 2);

				// Avoid tan(0) ==> NaN
				if ((tan_filt <= 0.0) && (tan_filt > -0.001))
					tan_filt = -0.001;
				if ((tan_filt >= 0.0) && (tan_filt < 0.001))
					tan_filt = 0.001;

				// Yay, new c
				c = 1 / tan_filt;

		}

		// Calculate new c for effort if we have a user specified cutoff frequency for effort
		if (cutoff_frequency_effort != 8){

				// Check if tan(_) is really small, could cause c = NaN
				tan_filt = tan((cutoff_frequency_effort * 6.2832) * delta_t.toSec() / 2);

				// Avoid tan(0) ==> NaN
				if ((tan_filt <= 0.0) && (tan_filt > -0.001))
					tan_filt = -0.001;
				if ((tan_filt >= 0.0) && (tan_filt < 0.001))
					tan_filt = 0.001;

				// Yay, new c
				c_effort = 1 / tan_filt;

		}

		// Assign derivatives of process value (velocity), make history to smoothen out
		pv_deriv_hist.col(2) = pv_deriv_hist.col(1);
		pv_deriv_hist.col(1) = pv_deriv_hist.col(0);
		for(int i=0; i<6; i++) { pv_deriv_hist(i,0) = qd_tmp(i); }

		// Make history to work with
		filtered_pv_deriv_hist.col(2) = filtered_pv_deriv_hist.col(1);
		filtered_pv_deriv_hist.col(1) = filtered_pv_deriv_hist.col(0);

		// Filter
		filtered_pv_deriv_hist.col(0) = (1 / (1 + c * c + 1.414 * c)) * (pv_deriv_hist.col(2) +
																																 2 * pv_deriv_hist.col(1) +
																																 		 pv_deriv_hist.col(0) -
																	(c * c - 1.414 * c + 1) * filtered_pv_deriv_hist.col(2) -
																				 (-2 * c * c + 2) * filtered_pv_deriv_hist.col(1));

		// Assign control effort history to smoothen out the control effort
		control_effort_hist.col(2) = control_effort_hist.col(1);
		control_effort_hist.col(1) = control_effort_hist.col(0);
		for(int i=0; i<6; i++) { control_effort_hist(i,0) = tau(i); }

		// Make history to work with
		filtered_control_effort_hist.col(2) = filtered_control_effort_hist.col(1);
		filtered_control_effort_hist.col(1) = filtered_control_effort_hist.col(0);

		// Filter
		filtered_control_effort_hist.col(0) = (1 / (1 + c_effort * c_effort + 1.414 * c_effort)) * (control_effort_hist.col(2) +
																																 			 											2 * control_effort_hist.col(1) +
																																  	 			 											control_effort_hist.col(0) -
																  			(c_effort * c_effort - 1.414 * c_effort + 1) * filtered_control_effort_hist.col(2) -
																	  		 			 				(-2 * c_effort * c_effort + 2) * filtered_control_effort_hist.col(1));

		// Nothing
		return;

}




// Function for assigning data to publish
void PD_w_g_Object::assignDataToPublish(Eigen::Vector6d joint_torque) { //https://eigen.tuxfamily.org/dox/group__TopicPassingByValue.html

		// Assign control effort to filter for processing
		for(int i=0; i<6; i++) control_effort_hist(i,0) = joint_torque(i);

		// Publish controll effort if controller is enabled
		if (pd_w_g_enabled) {

			// Make control effort good
			for (int i=0; i<6; i++) {

				// Applying saturation limits to filtered control effort
				if (filtered_control_effort_hist(i,0) > uL) filtered_control_effort_hist(i,0) = uL;
	    	else if (filtered_control_effort_hist(i,0) < lL) filtered_control_effort_hist(i,0) = lL;

				// Apply slope limiting criterion
				Slope(i) = ((filtered_control_effort_hist(i,0) - filtered_control_effort_hist(i,1))/delta_t.toSec());
				if (Slope(i) > 2) filtered_control_effort_hist(i,0) =  filtered_control_effort_hist(i,1) + Slope(i)*delta_t.toSec();

			}

			// Using standard messages
			std_msgs::Float64 msg;

			// Assigning the computed effort from the controller to each of the publishers
			msg.data = joint_torque(0);//filtered_control_effort_hist(0,0);
			pub_j1.publish(msg);
			msg.data = joint_torque(1);//filtered_control_effort_hist(1,0);
			pub_j2.publish(msg);
			msg.data = joint_torque(2);//filtered_control_effort_hist(2,0);
			pub_j3.publish(msg);
			msg.data = joint_torque(3);//filtered_control_effort_hist(3,0);
			pub_j4.publish(msg);
			msg.data = joint_torque(4);//filtered_control_effort_hist(4,0);
			pub_j5.publish(msg);
			msg.data = joint_torque(5);//filtered_control_effort_hist(5,0);
			pub_j6.publish(msg);

		}
		else ROS_WARN_STREAM("PD Controller with gravity compensation is not enabled");

		// https://www.youtube.com/watch?v=v2484MWr710
		return;

}




// Storing the data in the joint_state topic to our q and qd (velocity) vectors
void PD_w_g_Object::getJointStates(const sensor_msgs::JointState& joint_state) {

		// Please
		try {

					// Starting on i+simCounter because for simultaion /Crustcrawler/Joint_State is configured as: [gripper_left_joint, gripper_right_joint, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], as for hardware it is configured as  [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
					for (int i=0; i<6; i++) q(i) = joint_state.position[i+simCounter];

					// Storing velocities
					for (int i=0; i<6; i++) {

						// Only Rosie O'Donnell
						if (i < 5) qd(i) = joint_state.velocity[i+simCounter];
						else {

							// Awesome filtered stuff
							qd_tmp(i) = joint_state.velocity[i+simCounter];
							qd(i) = filtered_pv_deriv_hist(i,0);

						}

					}

		}
		catch (...) {
			ROS_ERROR("could not find joint_state, q or qd, at time: %f", ros::Time::now().toSec());

			// Go away
			return;
		}

}




// Print status to terminal
void PD_w_g_Object::printData(){

		// Terminalstuff
	  std::cout << "---------------Node name: " << ros::this_node::getName() << std::endl << std::endl;
		if (new_const_setpoint)
		std::cout << "----------Setpoint topic: " << setpoint_topic << std::endl << std::endl;
		else
		std::cout << "----------Setpoint topic: " << new_setpoint_topic << std::endl << std::endl;
		std::cout << "-Cutoff Frequency topics: " << cutoff_frequency_velocity_topic << std::endl;
		std::cout << "                          " << cutoff_frequency_effort_topic << std::endl << std::endl;
		std::cout << "-Topic from Crustcrawler: " << topic_from_crustcrawler << std::endl << std::endl;
		std::cout << "--Topics from Controller: " << topic1_from_controller << std::endl;
		std::cout	<< "                          " << topic2_from_controller << std::endl;
		std::cout	<< "                          " << topic3_from_controller << std::endl;
		std::cout	<< "                          " << topic4_from_controller << std::endl;
		std::cout	<< "                          " << topic5_from_controller << std::endl;
		std::cout	<< "                          " << topic6_from_controller << std::endl << std::endl;
		if (pd_w_g_enabled)
		std::cout << "------Controller Enabled: " << "True" << std::endl << std::endl;
		else
		std::cout << "------Controller Enabled: " << "False" << std::endl << std::endl;
		std::cout << "-------Saturation limits: " << uL << "/" << lL << std::endl << std::endl;
		std::cout << "----Cutoff Frequency Vel: " << cutoff_frequency_velocity << std::endl << std::endl;
		std::cout << "----Cutoff Frequency Eff: " << cutoff_frequency_effort << std::endl << std::endl;
		std::cout << "--------------c velocity: " << c << std::endl << std::endl;
		std::cout << "----------------c effort: " << c_effort << std::endl << std::endl;
		std::cout << "----------------------Kp: " << Kp_ << std::endl << std::endl;
		std::cout << "----------------------Kd: " << Kd_ << std::endl << std::endl;
		std::cout << "---Desired value Theta 1: " << q_d(0) << std::endl << std::endl;
		std::cout << "---Desired value Theta 2: " << q_d(1) << std::endl << std::endl;
		std::cout << "---Desired value Theta 3: " << q_d(2) << std::endl << std::endl;
		std::cout << "---Desired value Theta 4: " << q_d(3) << std::endl << std::endl;
		std::cout << "---Desired value Theta 5: " << q_d(4) << std::endl << std::endl;
		std::cout << "---Desired value Theta 6: " << q_d(5) << std::endl << std::endl;
		std::cout << "----------Error---------- " << std::endl << std::endl;
		std::cout << std::setprecision(5) << (q_d-q) << std::endl << std::endl;
		std::cout << "------Control Effort----- " << std::endl << std::endl;
		std::cout << std::setprecision(5) << tau  << std::endl;

		// You are coming back, right?
		return;

}




// Check that we have reasonable saturation limits
bool PD_w_g_Object::validateParameters(){

		// Lower limit can obv not be greater then upper limit
	  if (lL > uL){
	    ROS_ERROR("The lower saturation limit cannot be greater than the upper saturation limit.");
	    return (false);
	  }

		// This is so true
	  return true;

}




// Function for getting the joint state and calculating effort provided by the PD controller with gravity compensation
void PD_w_g_Object::callbackJointState(const sensor_msgs::JointState& joint_state) {

		// Assign joint states to vectors
		PD_w_g_Object::getJointStates(joint_state);

		// Setting Proportional and Derivative gain
		K_P.setConstant(Kp_);
		K_D.setConstant(Kd_);

		// PD controller with gravity compensation
		tau = getGravity(q) + K_P.asDiagonal()*(q_d-q) - K_D.asDiagonal()*qd;

		// Prinitng out data and display message if saturation limits are wrong
		PD_w_g_Object::printData();
  	if (not validateParameters()) std::cout << "Error: invalid parameters for Saturation limits!\n";

		// Apply saturation limits to control effort
		for (int i=0; i++; i<6) {

			//https://www.youtube.com/watch?v=K91imfIWMV4
			if (tau(i) > uL) tau(i) = uL;
    	else if (tau(i) < lL) tau(i) = lL;

		}

		// Sometimes I believe compiler ignores all my comments
		PD_w_g_Object::assignDataToPublish(tau);

		// https://youtu.be/FzOoudy8u2E?t=32
		return;

}




// Function for getting parameters in the first reconfiguration
void PD_w_g_Object::getParameters(double in, double& value, double& scale){

		// x to the power of digits
	  int digits = 0;

		// Assign to value passed by reference
		value = in;

		// Make sure that the value is within what we want it to be
	  while ((fabs(value) > 1.0 || fabs(value) < 0.1) && (digits < 2 && digits > -1)){

				if (fabs(value) > 1.0){

					// If you know the logic of += you know the logic of /=
		      value /= 10.0;
		      digits++;

				}
		    else{

					// Random comment
		      value *= 10.0;
		      digits--;

				}

	  }

		// Only needs values between -1 and 1
	  if (value > 1.0) value = 1.0;
	  if (value < -1.0) value = -1.0;

		// Assigning proper scale to the scale parameter passed by reference
	  scale = pow(10.0, digits);

		// This only happens once, so it's not that exciting
		return;

}




// Function to reconfigure the Kp and Kd values
void PD_w_g_Object::reconfigureCallback(crustcrawler_controllers::pd_w_gConfig& config, uint32_t level){

		// Set value and scale first time
	  if (first_reconfig){
	    getParameters(Kp_, config.Kp, config.Kp_scale);
	    getParameters(Kd_, config.Kd, config.Kd_scale);
	    first_reconfig = false;

			// Ignore the first call to reconfigure which happens at startup
	    return;

	  }

		// Update Kp and Kd
	  Kp_ = config.Kp * config.Kp_scale;
	  Kd_ = config.Kd * config.Kd_scale;
	  ROS_INFO("PD_w_g reconfigure request: Kp: %f, Kd: %f", Kp_, Kd_);

		// No comment for you
		return;

}




// Constructor stuff
PD_w_g_Object::PD_w_g_Object(ros::NodeHandle &nh, ros::NodeHandle &nh_priv) : pv_deriv_hist(6,3), control_effort_hist(6,3),  filtered_pv_deriv_hist(6,3), filtered_control_effort_hist(6,3){


		// Everybody loves information, especially from ROS
		while ( ros::ok() && ros::Time(0) == ros::Time::now() ){
			ROS_INFO("Controller spinning, waiting for time to become non-zero");
			sleep(1);
		}


		// Get params if specified in launch file or as params on command-line, set defaults
		nh_priv.param<double>("/Kp", Kp_, 2.0);
		nh_priv.param<double>("/Kd", Kd_, 0.5);
		nh_priv.param<double>("/upper_limit", uL, 5.0);
		nh_priv.param<double>("/lower_limit", lL, -5.0);
		nh_priv.param<double>("cutoff_frequency_velocity", cutoff_frequency_velocity, 0.001);
		nh_priv.param<std::string>("topic1_from_controller", topic1_from_controller, "/crustcrawler/joint1_controller/command");
		nh_priv.param<std::string>("topic2_from_controller", topic2_from_controller, "/crustcrawler/joint2_controller/command");
		nh_priv.param<std::string>("topic3_from_controller", topic3_from_controller, "/crustcrawler/joint3_controller/command");
		nh_priv.param<std::string>("topic4_from_controller", topic4_from_controller, "/crustcrawler/joint4_controller/command");
		nh_priv.param<std::string>("topic5_from_controller", topic5_from_controller, "/crustcrawler/joint5_controller/command");
		nh_priv.param<std::string>("topic6_from_controller", topic6_from_controller, "/crustcrawler/joint6_controller/command");
		nh_priv.param<std::string>("topic_from_crustcrawler", topic_from_crustcrawler, "/crustcrawler/joint_states");
		nh_priv.param<std::string>("cutoff_frequency_velocity_topic", cutoff_frequency_velocity_topic, "/crustcrawler/pd_w_g/cutoff_frequency_velocity");
		nh_priv.param<std::string>("cutoff_frequency_effort_topic", cutoff_frequency_effort_topic, "/crustcrawler/pd_w_g/cutoff_frequency_effort");
		nh_priv.param<std::string>("new_setpoint_topic", new_setpoint_topic, "/crustcrawler/pd_w_g/new_setpoint/");
		nh_priv.param<std::string>("setpoint_topic", setpoint_topic, "/crustcrawler/pd_w_g/setpoint/");
		nh_priv.param<std::string>("enable_topic", enable_topic, "/crustcrawler/pd_w_g/enable");


		// Making sure we read the right part of the Joint_state sensor_msg from hardware node
		nh_priv.getParam("/simulation", simulation_toggle);
		if (!simulation_toggle) simCounter = 0;


		// Dynamic reconfiguration - Do not touch (Magic)
		dynamic_reconfigure::Server<crustcrawler_controllers::pd_w_gConfig> server;
		dynamic_reconfigure::Server<crustcrawler_controllers::pd_w_gConfig>::CallbackType f;
		f = boost::bind(&PD_w_g_Object::reconfigureCallback, this, _1, _2);
		server.setCallback(f);


		// Subscribing to relevant topics
		sub_pd_w_g_enabled = nh.subscribe(enable_topic, 1, &PD_w_g_Object::pd_w_g_EnableCallback, this);																	// Boolean published to turn on/off the controller
		sub_setpoint = nh.subscribe(setpoint_topic, 1, &PD_w_g_Object::setpointCallback, this);																						// Double that is set as all the desired joint values equal
		sub_new_setpoint = nh.subscribe(new_setpoint_topic, 1, &PD_w_g_Object::newSetpointCallback, this);																// Joint_state sensor_msgs with a vector of individual joint values
		sub_joint_states = nh.subscribe(topic_from_crustcrawler, 1, &PD_w_g_Object::callbackJointState, this);														// Getting Joint states and assigning data to publish
		sub_cutoff_frequency_velocity = nh.subscribe(cutoff_frequency_velocity_topic, 1, &PD_w_g_Object::cutoffVelocityCallback, this);		// Getting new cutoff frequency if needed
		sub_cutoff_frequency_effort = nh.subscribe(cutoff_frequency_effort_topic, 1, &PD_w_g_Object::cutoffEffortCallback, this);					// Getting new cutoff frequency if needed


		// Publishing on topics for joint control on crustcrawler
		pub_j1 = nh.advertise<std_msgs::Float64>(topic1_from_controller, 1);
		pub_j2 = nh.advertise<std_msgs::Float64>(topic2_from_controller, 1);
		pub_j3 = nh.advertise<std_msgs::Float64>(topic3_from_controller, 1);
		pub_j4 = nh.advertise<std_msgs::Float64>(topic4_from_controller, 1);
		pub_j5 = nh.advertise<std_msgs::Float64>(topic5_from_controller, 1);
		pub_j6 = nh.advertise<std_msgs::Float64>(topic6_from_controller, 1);


		// Wait for the first setpoint and Joint state messages
		while( ros::ok() && !ros::topic::waitForMessage<std_msgs::Float64>(setpoint_topic, ros::Duration(10.)) ) ROS_WARN_STREAM("Waiting for the first setpoints");
		while( ros::ok() && !ros::topic::waitForMessage<sensor_msgs::JointState>(topic_from_crustcrawler, ros::Duration(10.)) ) ROS_WARN_STREAM("Waiting for the first joint state message from Crustcrawler");


		// Run until shut down
		while (ros::ok()){

				// Running the low pass filter only on the velocity of theta6
				lowPassFilter();

				// Pumping callbacks
				ros::spinOnce();

				// A small sleep (for man) to avoid 100% CPU usage (for mankind)
				ros::Duration(0.001).sleep();

		}

};
