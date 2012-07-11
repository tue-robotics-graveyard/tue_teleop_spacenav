#include <ros/ros.h>
//#include <joy/Joy.h>
#include <math.h>
#include "sensor_msgs/Joy.h"

#include <geometry_msgs/Twist.h>
#include <amigo_msgs/head_ref.h>
#include <amigo_kinematics/set_Arm.h>

// Global variables
bool pressed;
double init_time;
double last_time;
uint mode = 0;

double minimum_movement;
double joystick_range;
double max_translational_speed;
double max_angular_speed;

ros::Publisher vel_pub;
ros::Publisher head_pub;
//ros::ServiceClient arm_left_pub;
//ros::ServiceClient arm_right_pub;
ros::Publisher arm_left_pub;
ros::Publisher arm_right_pub;

// Arms:
double lin_scale = 0.05;
double ang_scale = 0.1;

double prev_left_pos[6] = { 0.4, 0.2, 0.8, 0.0, 0.0, 0.0 };
double prev_right_pos[6] = { 0.4, -0.2, 0.8, 0.0, 0.0, 0.0 };

void SpaceNavCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    double current_time = ros::WallTime::now().toSec() - init_time;
    //double dt = current_time - last_time; //Only required when using the service calls for the arms
    last_time = current_time;

    bool button1 = msg->buttons[0];
    bool button2 = msg->buttons[1];

    // Normalise joystick commands:
    double dof[6];
    for ( uint i = 0; i < 6; i++ )
    {
        dof[i] = msg->axes[i] / joystick_range;
        if ( dof[i] > minimum_movement )
            dof[i] = ( dof[i] - minimum_movement )/( 1 - minimum_movement );
        else if ( dof[i] < -minimum_movement )
            dof[i] = ( dof[i] + minimum_movement )/( 1 - minimum_movement );
        else
            dof[i] = 0;
    }
    ROS_DEBUG("x = %f  \t  y = %f  \t  z = %f  \t  x_ang = %f  \t  y_ang = %f  \t  z_ang = %f  \t  ", dof[0], dof[1], dof[2], dof[3], dof[4], dof[5]);


    // Switch between modes:
    if (button1){
        if (mode <= 3 && !pressed){
            mode++;
            ROS_INFO("Mode = %u",mode);
        }
        else if (!pressed)
        {
            mode = 0;
            ROS_INFO("Mode = %u",mode);
        }
        pressed = true;
    }
    else
        pressed = false;

    // Switch between operating modes:
    switch(mode)
    {
        case 1:
        {
            ROS_INFO("Moving head mode");
            double pan = dof[5];
            double tilt = dof[4];

            double angle_factor = 1;
            if (button2)
            {
                ROS_INFO("Bigger angle");
                angle_factor = 2.5;
            }
            amigo_msgs::head_ref head_msg;
            head_msg.head_pan = pan*angle_factor;
            head_msg.head_tilt = tilt;
            head_pub.publish(head_msg);
            ROS_DEBUG("pan = %f \t tilt = %f \t ",head_msg.head_pan,head_msg.head_tilt);
            break;
        }
        case 2:
        {
            ROS_INFO("Moving base mode");

            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = max_translational_speed * dof[0];
            vel_msg.linear.y = max_translational_speed * dof[1];
            vel_msg.linear.z = max_translational_speed * dof[2];
            vel_msg.angular.z = max_angular_speed * dof[5];

            vel_pub.publish(vel_msg);
            break;
        }
        case 3:
        {
            ROS_INFO("Moving left arm mode");

            /*
		//numerically integrate
		double pos[6];
		for ( uint i = 0; i < 3; i++ )
			pos[i] = prev_left_pos[i] + lin_scale * dof[i] * dt;
		for ( uint i = 3; i < 6; i++ )
			pos[i] = prev_left_pos[i] + ang_scale * dof[i] * dt;

		ROS_INFO("x = %f  \t y = %f \t z = %f \t r = %f \t p = %f \t y = %f \t dt = %f \t ", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], dt );

		amigo_kinematics::set_Arm arm_srv;
		arm_srv.request.x = pos[0];
		arm_srv.request.y = pos[1];
		arm_srv.request.z = pos[2];
		arm_srv.request.roll = pos[3];
		arm_srv.request.pitch = pos[4];
		arm_srv.request.yaw = pos[5];

		bool change = false;
		for ( uint i = 0; i < 3; i++ )
			if (pos[i] != prev_left_pos[i])
				change = true;

		if (change)
		{
			if (arm_left_pub.call(arm_srv))
			{
				if (arm_srv.response.ik_valid)
				{
					ROS_INFO("Jeej");
					for ( uint i = 0; i < 6; i++ )
						prev_left_pos[i] = pos[i];
				}
				else
				{
					ROS_INFO("Invalid position.");
				}
			}
			else
			{
				ROS_ERROR("Failed to call service set_Arm");
			}
		}
             */

            ROS_INFO("vx = %f  \t vy = %f \t vz = %f \t vr = %f \t vp = %f \t vy = %f ", lin_scale * dof[0], lin_scale * dof[1], lin_scale * dof[2], ang_scale * dof[3], ang_scale * dof[4], ang_scale * dof[5]);

            geometry_msgs::Twist arm_msg;

            arm_msg.linear.x = lin_scale * dof[0];
            arm_msg.linear.y = lin_scale * dof[1];
            arm_msg.linear.z = lin_scale * dof[2];
            arm_msg.angular.x = ang_scale * dof[3];
            arm_msg.angular.y = ang_scale * dof[4];
            arm_msg.angular.z = ang_scale * dof[5];

            arm_left_pub.publish(arm_msg);

            break;
        }
        case 4:
        {
            ROS_INFO("Moving right arm mode");

            /*
            //numerically integrate
            double pos[6];
            for ( uint i = 0; i < 6; i++ )
                pos[i] = prev_right_pos[i] + dof[i] * dt;

            ROS_INFO("x = %f  \t y = %f \t z = %f \t r = %f \t p = %f \t y = %f \t dt = %f \t ", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], dt );

            amigo_kinematics::set_Arm arm_srv;
            arm_srv.request.x = pos[0];
            arm_srv.request.y = pos[1];
            arm_srv.request.z = pos[2];
            arm_srv.request.roll = -pos[3];
            arm_srv.request.pitch = -pos[4];
            arm_srv.request.yaw = -pos[5];

            if (arm_right_pub.call(arm_srv))
            {
                if (arm_srv.response.ik_valid)
                {
                    ROS_INFO("Jeej");
                    for ( uint i = 0; i < 6; i++ )
                        prev_right_pos[i] = pos[i];
                }
                else
                {
                    ROS_INFO("Invalid position.");
                }
            }
            else
            {
                ROS_ERROR("Failed to call service set_Arm");
            }
             */

            ROS_INFO("vx = %f  \t vy = %f \t vz = %f \t vr = %f \t vp = %f \t vy = %f ", lin_scale * dof[0], lin_scale * dof[1], lin_scale * dof[2], ang_scale * dof[3], ang_scale * dof[4], ang_scale * dof[5]);

            geometry_msgs::Twist arm_msg;

            arm_msg.linear.x = lin_scale * dof[0];
            arm_msg.linear.y = lin_scale * dof[1];
            arm_msg.linear.z = lin_scale * dof[2];
            arm_msg.angular.x = ang_scale * dof[3];
            arm_msg.angular.y = ang_scale * dof[4];
            arm_msg.angular.z = ang_scale * dof[5];

            arm_right_pub.publish(arm_msg);

            break;
        }
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "spacenav_publisher");

    //Get namespace
    std::string ns = ros::this_node::getName();

    ros::NodeHandle n;
    ros::Subscriber spacenav_sub = n.subscribe("/spacenav/joy", 1, &SpaceNavCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    head_pub = n.advertise<amigo_msgs::head_ref>("/head_controller/set_Head", 1);
    //arm_left_pub = n.serviceClient<amigo_kinematics::set_Arm>("/arm_left/set_tip");
    //arm_right_pub = n.serviceClient<amigo_kinematics::set_Arm>("/arm_right/set_tip");
    arm_left_pub = n.advertise<geometry_msgs::Twist>("/arm_left_controller/cartesian_velocity_reference",1);
    arm_right_pub = n.advertise<geometry_msgs::Twist>("/arm_right_controller/cartesian_velocity_reference",1);

    init_time = ros::WallTime::now().toSec();

    //Load parameters
    n.param<double> (ns+"/minimum_movement", minimum_movement, 0.2);
    n.param<double> (ns+"/joystick_range", joystick_range, 0.68);
    n.param<double> (ns+"/max_linear_speed", max_translational_speed, 0.5);
    n.param<double> (ns+"/max_angular_speed", max_angular_speed, 0.8);

    ros::Rate rate(15);
    while (n.ok()){

        ros::spinOnce();
        rate.sleep();
    }
    return 1;

}
