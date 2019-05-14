/******************************************************************************************
*
*
* PD controller with  gravity compensation and dynamic reconfigure
* Date sometime in July 2018
* If this code works, it was written by Eirik Kvalheim. If not, I don’t know who wrote it.
*
*
******************************************************************************************/


#include <pd_w_g/pd_controller_w_gravity_and_dynamic_reconfigure.h>
#include <iostream>


// https://www.youtube.com/watch?v=n7cEWfskTUs
int main(int argc, char **argv){

		// Command line parameters and name of the node
		ros::init(argc, argv, "pd_controller_w_gravity");

		// Main access points to communication with ROS
		ros::NodeHandle nh;
		ros::NodeHandle nh_priv("~");

		// An instance of the PD_w_g_Object
    crustcrawler::PD_w_g_Object tek4030_PD_Controller_Instance(nh, nh_priv);

		// https://www.youtube.com/watch?v=i92RcDQBOyA
		return 0;

}


/*


To run controller, run these commands in secquenced order:


For real hardware:

roslaunch crustcrawler_hardware control.launch full_arm:=true tek4030:=true  gripper_enabled:=false control:=position
rostopic pub -1 /crustcrawler/enable std_msgs/Bool true
roslaunch crustcrawler_controllers pd_w_g.launch simulation:=false


For simulation:

roslaunch crustcrawler_gazebo controller.launch tek4030:=true
roslaunch crustcrawler_controllers pd_w_g.launch simulation:=true


If you want to stop the controller withouth killing any nodes, you can run;
rostopic pub -1 /crustcrawler/pd_w_g/enable std_msgs/Bool false

You can also set the setpoint from terminal by running for example;
rostopic pub /crustcrawler/pd_w_g/setpoint/ std_msgs/Float64 "data: 0.1"

rostopic pub /crustcrawler/pd_w_g/cutoff_frequency_effort std_msgs/Float64 "data: 1.0"




If you have problems running the codes, try:

sudo apt install ros-melodic-gazebo-ros-control ros-melodic-ros-control ros-melodic-ros-controllers


*/
