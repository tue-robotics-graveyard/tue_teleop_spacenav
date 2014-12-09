#include <ros/ros.h>
//#include <joy/Joy.h>
#include <math.h>
#include "sensor_msgs/Joy.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>

// Global variables
double loop_rate = 15;

bool pressed;
double init_time;
double last_time;
uint mode = 0;
uint previous_mode = 0;

double minimum_movement;
double joystick_range;
double max_linear_speed_base;
double max_angular_speed_base;

ros::Publisher vel_pub;
ros::Publisher head_pub;

ros::Publisher torso_pub;
ros::Publisher arm_left_pub;
ros::Publisher arm_right_pub;

// Torso
double max_torso_speed = 0.1;
double current_torso_pos = 0.35; //0.35 is torso height at startup

// Arms:
double max_linear_speed_arms = 0.05;
double max_angular_speed_arms = 0.1;

double prev_left_pos[6] = { 0.4, 0.2, 0.8, 0.0, 0.0, 0.0 };
double prev_right_pos[6] = { 0.4, -0.2, 0.8, 0.0, 0.0, 0.0 };

void SpaceNavCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    double current_time = ros::WallTime::now().toSec() - init_time;
    //double dt = current_time - last_time; //Only required when using the service calls for the arms
    //last_time = current_time;

    bool button1 = msg->buttons[0];
    bool button2 = msg->buttons[1];

    // Normalise joystick commands:
    double dof[6];
    for ( uint i = 0; i < 6; i++ )
    {
        dof[i] = msg->axes[i] / joystick_range;
        if ( dof[i] > minimum_movement ) {
            dof[i] = ( dof[i] - minimum_movement )/( 1 - minimum_movement );
            last_time = ros::Time::now().toSec() - init_time;
        } else if ( dof[i] < -minimum_movement ) {
            dof[i] = ( dof[i] + minimum_movement )/( 1 - minimum_movement );
            last_time = ros::Time::now().toSec() - init_time;
        } else {
            dof[i] = 0;
		}
    }
    ROS_DEBUG("x = %f  \t  y = %f  \t  z = %f  \t  x_ang = %f  \t  y_ang = %f  \t  z_ang = %f  \t  ", dof[0], dof[1], dof[2], dof[3], dof[4], dof[5]);
    
    // Safety/convenience: switch off after 10 seconds
    if (pressed) {
		last_time = ros::Time::now().toSec() - init_time;
	}
    if ( (current_time - last_time) > 10.0 && mode != 0) {
		ROS_INFO("Spacenav: no input for 10 seconds, going to idle");
		mode = 0;
	}

    // Switch between modes:
    if (button1){
        if (mode <= 2 && !pressed){
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
            if (mode != previous_mode) ROS_INFO("Moving head mode");
            double pan = dof[5];
            double tilt = dof[4];

            double angle_factor = 1;
            if (button2)
            {
                ROS_INFO("Bigger angle");
                angle_factor = 2.5;
            }
            sensor_msgs::JointState head_msg;
            head_msg.name.push_back("neck_pan_joint");
            head_msg.name.push_back("neck_tilt_joint");
            head_msg.position.push_back(pan*angle_factor);
            head_msg.position.push_back(tilt);
            head_pub.publish(head_msg);
            ROS_DEBUG("pan = %f \t tilt = %f \t ",head_msg.position[0],head_msg.position[1]);
            break;
        }
        case 2:
        {
            if (mode != previous_mode) ROS_INFO("Moving base mode");

            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = max_linear_speed_base * dof[0];
            vel_msg.linear.y = max_linear_speed_base * dof[1];
            vel_msg.linear.z = max_linear_speed_base * dof[2];
            vel_msg.angular.z = max_angular_speed_base * dof[5];

            vel_pub.publish(vel_msg);
            break;
        }

        case 3:
        {

            if (mode != previous_mode) ROS_INFO("Moving torso mode");

            sensor_msgs::JointState torso_msg;
            torso_msg.name.push_back("torso_joint");
            torso_msg.position.push_back(current_torso_pos + max_torso_speed * dof[2] / loop_rate);
            torso_msg.velocity.push_back(max_torso_speed * dof[2]);
            
			//ROS_INFO("Torso speed = %f", max_torso_speed * dof[2]);
			
            torso_pub.publish(torso_msg);
            break;

        }

        case 4:
        {
            if (mode != previous_mode) ROS_INFO("Moving left arm mode");

            //ROS_INFO("vx = %f  \t vy = %f \t vz = %f \t vr = %f \t vp = %f \t vy = %f ", max_linear_speed_arms * dof[0], max_linear_speed_arms * dof[1], max_linear_speed_arms * dof[2], max_angular_speed_arms * dof[3], max_angular_speed_arms * dof[4], max_angular_speed_arms * dof[5]);

            geometry_msgs::TwistStamped arm_msg;

			arm_msg.header.frame_id = "base_link";
            arm_msg.twist.linear.x = max_linear_speed_arms * dof[0];
            arm_msg.twist.linear.y = max_linear_speed_arms * dof[1];
            arm_msg.twist.linear.z = max_linear_speed_arms * dof[2];
            arm_msg.twist.angular.x = max_angular_speed_arms * dof[3];
            arm_msg.twist.angular.y = max_angular_speed_arms * dof[4];
            arm_msg.twist.angular.z = max_angular_speed_arms * dof[5];

            arm_left_pub.publish(arm_msg);

            break;
        }
        case 5:
        {
            if (mode != previous_mode) ROS_INFO("Moving right arm mode");

            //ROS_INFO("vx = %f  \t vy = %f \t vz = %f \t vr = %f \t vp = %f \t vy = %f ", max_linear_speed_arms * dof[0], max_linear_speed_arms * dof[1], max_linear_speed_arms * dof[2], max_angular_speed_arms * dof[3], max_angular_speed_arms * dof[4], max_angular_speed_arms * dof[5]);

            geometry_msgs::TwistStamped arm_msg;

			arm_msg.header.frame_id = "base_link";
            arm_msg.twist.linear.x = max_linear_speed_arms * dof[0];
            arm_msg.twist.linear.y = max_linear_speed_arms * dof[1];
            arm_msg.twist.linear.z = max_linear_speed_arms * dof[2];
            arm_msg.twist.angular.x = max_angular_speed_arms * dof[3];
            arm_msg.twist.angular.y = max_angular_speed_arms * dof[4];
            arm_msg.twist.angular.z = max_angular_speed_arms * dof[5];

            arm_right_pub.publish(arm_msg);

            break;
        }
    }
    
    previous_mode = mode;

}

void TorsoCallback(const sensor_msgs::JointState::ConstPtr& msg) {

    // Fill in data in spindle position
    current_torso_pos = msg->position[0];

}

int main(int argc, char** argv){
    ros::init(argc, argv, "spacenav_publisher");

    //Get namespace
    std::string ns = ros::this_node::getName();

    ros::NodeHandle n;
    ros::Subscriber spacenav_sub = n.subscribe("/spacenav/joy", 1, &SpaceNavCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    head_pub = n.advertise<sensor_msgs::JointState>("/neck/references", 1);

    ros::Subscriber torso_sub = n.subscribe("/torso/measurements", 1, &TorsoCallback);
    torso_pub = n.advertise<sensor_msgs::JointState>("/torso/references", 1);

    arm_left_pub = n.advertise<geometry_msgs::TwistStamped>("/arm_left_controller/cartesian_velocity_reference",1);
    arm_right_pub = n.advertise<geometry_msgs::TwistStamped>("/arm_right_controller/cartesian_velocity_reference",1);

    init_time = ros::WallTime::now().toSec();

    //Load parameters
    n.param<double> (ns+"/minimum_movement", minimum_movement, 0.2);
    n.param<double> (ns+"/joystick_range", joystick_range, 0.68);
    n.param<double> (ns+"/max_linear_speed_base", max_linear_speed_base, 0.5);
    n.param<double> (ns+"/max_angular_speed_base", max_angular_speed_base, 0.8);
    n.param<double> (ns+"/max_linear_speed_arms", max_linear_speed_arms, 0.05);
    n.param<double> (ns+"/max_angular_speed_arms", max_angular_speed_arms, 0.1);
    n.param<double> (ns+"/max_torso_speed", max_torso_speed, 0.5);

    ros::Rate rate(loop_rate);
    while (n.ok()){

        ros::spinOnce();
        rate.sleep();
    }
    return 1;

}
