#include <ros/ros.h>
#include <joy/Joy.h>
#include <math.h>
#include "sensor_msgs/Joy.h"

#include <geometry_msgs/Twist.h>

// Defined variables: //@TODO: Create parameters
#define minumum_movement 0.15
#define joystick_range 0.68
#define max_translational_speed 0.5
#define max_angular_speed 0.8



ros::Publisher vel_pub;


void SpaceNavCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

	bool button1 = msg->buttons[0];
	bool button2 = msg->buttons[1];

	if ( button1 && button2 )
	{
		// Normalise joystick commands:
		double dof[6];
		for ( uint i = 0; i < 6; i++ )
		{
			dof[i] = msg->axes[i] / joystick_range;
			if ( dof[i] > minumum_movement )
				dof[i] = ( dof[i] - minumum_movement )/( 1 - minumum_movement );
			else if ( dof[i] < -minumum_movement )
				dof[i] = ( dof[i] + minumum_movement )/( 1 - minumum_movement );
			else
				dof[i] = 0;
		}
		ROS_DEBUG("x = %f  \t  y = %f  \t  z = %f  \t  x_ang = %f  \t  y_ang = %f  \t  z_ang = %f  \t  ", dof[0], dof[1], dof[2], dof[3], dof[4], dof[5]);

		geometry_msgs::Twist vel_msg;
		vel_msg.linear.x = max_translational_speed * dof[0];
		vel_msg.linear.y = max_translational_speed * dof[1];
		vel_msg.linear.z = max_translational_speed * dof[2];
		vel_msg.angular.z = max_angular_speed * dof[5];

		vel_pub.publish(vel_msg);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "spacenav_publisher");

	ros::NodeHandle n;
	ros::Subscriber spacenav_sub = n.subscribe("/spacenav/joy", 1, &SpaceNavCallback);
	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	ros::Rate rate(15);
	while (n.ok()){

		ros::spinOnce();
		rate.sleep();
	}
	return 1;

}
