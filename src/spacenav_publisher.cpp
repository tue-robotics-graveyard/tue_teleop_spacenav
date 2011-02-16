#include <ros/ros.h>
#include <joy/Joy.h>
#include <math.h>

#include <geometry_msgs/Twist.h>


double x_min=0.3;
double y_min=0.3;
double z_min=0.3;

double x_max=0.5;
double y_max=0.5;
double z_max=0.5;
double z_ang_max=0.8;


double init_time;
double current_time;

double x_range=0.68;
double y_range=0.68;
double z_range=0.68;
double z_ang_range=0.68;

ros::Publisher vel_pub;
geometry_msgs::Twist vel_msg;
int mode=1;
double timer=0;
double check=0.1;
double linear_x;
double linear_y;
double linear_z;
double angular_z;




void SpaceNavCallback(const joy::Joy::ConstPtr& msg)
{  current_time = ros::WallTime::now().toSec()-init_time;
   
   int button=msg->buttons[0];
      check=ros::WallTime::now().toSec()-init_time-timer;
   if (button==1){

      if (mode==1 && check>=0.5){
          mode=2;
          timer=ros::WallTime::now().toSec()-init_time;
      }
      else if (mode==2 && check>=0.5){
          mode=1;
          timer=ros::WallTime::now().toSec()-init_time;
      }
   };

   
   // moving base mode
   if (mode==2){
   ROS_INFO("Moving base mode");

  linear_x=msg->axes[0]/x_range;
   if (linear_x>x_min){
      linear_x=(linear_x-x_min)/(1-x_min);
      }
   else if (linear_x<-x_min){ 
     linear_x=(linear_x+x_min)/(1-x_min);
     } 
   else
   {linear_x=0;}


  linear_y=msg->axes[1]/y_range;
   if (linear_y>y_min){
      linear_y=(linear_y-y_min)/(1-y_min);
      }
   else if (linear_y<-y_min){ 
     linear_y=(linear_y+y_min)/(1-y_min);
     } 
   else
   {linear_y=0;}

 
  linear_z=msg->axes[2]/z_range;
   if (linear_z>z_min){
      linear_z=(linear_z-z_min)/(1-z_min);
      }
   else if (linear_z<-z_min){ 
     linear_z=(linear_z+z_min)/(1-z_min);
     } 
   else
   {linear_z=0;}


   angular_z=msg->axes[5]/z_ang_range;
   if (angular_z>z_min){
      angular_z=(angular_z-z_min)/(1-z_min);
      }
   else if (angular_z<-z_min){ 
     angular_z=(angular_z+z_min)/(1-z_min);
     } 
   else
   {angular_z=0;}


   vel_msg.linear.x=x_max*linear_x;
   vel_msg.linear.y=y_max*linear_y;
   vel_msg.linear.z=z_max*linear_z;
   vel_msg.angular.z=z_ang_max*angular_z;

   vel_pub.publish(vel_msg);
   ROS_INFO("x=%f\ty=%f\tz=%f\tz_ang=%f\t",vel_msg.linear.x,vel_msg.linear.y,vel_msg.linear.z,vel_msg.angular.z);
   };


   
}



int main(int argc, char** argv){
  ros::init(argc, argv, "spacenav_publisher");

  ros::NodeHandle n;
  ros::Subscriber spacenav_sub = n.subscribe("/spacenav/joy", 1, &SpaceNavCallback);
  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Rate rate(1000);
  ros::WallTime current_time2;
  init_time = ros::WallTime::now().toSec();
  while (n.ok()){

  ros::spinOnce();
  rate.sleep();
  }
  return 1;

}
