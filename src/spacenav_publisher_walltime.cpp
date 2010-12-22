#include <ros/ros.h>
#include <joy/Joy.h>
#include <math.h>


#include <geometry_msgs/Twist.h>
#include <amigo_msgs/tip_ref.h>

double xd=0.0;
double yd=0.0;
double zd=0.0;
double rolld=0.0;
double pitchd=0.0;
double yawd=0.0;
double current_time;
double last_time;
double dt=0;
double x=0;
double y=0;
double z=0;
double roll=0;
double pitch=0;
double yaw=0;
double x_min=0.014;
double z_min=0.1;
double init_time;

double x_range=0.059;
double y_range=0.059;
double z_range=0.059;
double z_base_range=0.15;
double roll_range=0.059;
double pitch_range=0.059;
double yaw_range=0.059;
ros::Publisher cart_pub;
ros::Publisher vel_pub;
amigo_msgs::tip_ref cart_msg;
geometry_msgs::Twist vel_msg;
int mode=1;
double timer=0;
double check=0.1;
double linear_x;
double linear_y;
double linear_z;
double angular_z;
double vx=0;
double vy=0;
double vz=0;
double acc_lin=0.02;
double acc_ang=0.1;
double scale=0.1;
double scale_lin=5.5;
double scale_ang=7;




void SpaceNavCallback(const joy::Joy::ConstPtr& msg)
{  current_time = ros::WallTime::now().toSec()-init_time;
   dt=current_time-last_time;

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



   //Arm kinematics mode
   if (mode==1){
   ROS_INFO("End effector mode");
   if (dt>0.05){
        xd=0;
        yd=0;
        zd=0;
        rolld=0;
        pitchd=0;
        yawd=0;
   }
   else
   {}
   //set thresholds for reducing the sensitivity
   xd=-1*scale*(msg->axes[0]);
  //ROS_INFO("xd%f\t",xd);
   if (xd>x_min){
      xd=(xd-x_min)*x_range/(x_range-x_min);
      }
   else if (xd<-x_min){ 
     xd=(xd+x_min)*x_range/(x_range-x_min);
     } 
   else
   {xd=0;}

   yd=-1*scale*(msg->axes[1]);
  //ROS_INFO("yd%f\t",yd);
   if (yd>x_min ){
      yd=(yd-x_min)*y_range/(y_range-x_min);
      }
   else if (yd<-x_min){ 
     yd=(yd+x_min)*y_range/(y_range-x_min);
     } 
   else
   {yd=0;}

   zd=-1*scale*(msg->axes[2]);
   //ROS_INFO("zd%f\t",zd);
   if (zd>x_min){
   zd=(zd-x_min)*z_range/(z_range-x_min);
      }
   else if (zd<-x_min){ 
     zd=(zd+x_min)*z_range/(z_range-x_min);
     } 
   else
   {zd=0;}

   rolld=-1*scale*(msg->axes[3]);
   //ROS_INFO("rolld%f\t",rolld);
   if (rolld>x_min){
   rolld=(rolld-x_min)*roll_range/(roll_range-x_min);
      }
   else if (rolld<-x_min){ 
    rolld=(rolld+x_min)*roll_range/(roll_range-x_min);
     }
   else
   {rolld=0;}

   pitchd=-1*scale*(msg->axes[4]);
   //ROS_INFO("pitchd%f\t",pitchd);
   if (pitchd>x_min){
   pitchd=(pitchd-x_min)*pitch_range/(pitch_range-x_min);
      }
   else if (pitchd<-x_min){ 
    pitchd=(pitchd+x_min)*pitch_range/(pitch_range-x_min);
     }      
   else
   {pitchd=0;}

   yawd=-1*scale*(msg->axes[5]);
   //ROS_INFO("yawd%f\t",yawd);
   if (yawd>x_min){
   yawd=(yawd-x_min)*yaw_range/(yaw_range-x_min);
      }
   else if (yawd<-x_min){ 
    yawd=(yawd+x_min)*yaw_range/(yaw_range-x_min);
     }       
   else
   {yawd=0;}

   //numerically integrate
   x+=xd*dt;
   y+=yd*dt;
   z+=zd*dt;
   roll+=rolld*dt;
   pitch+=pitchd*dt;
   yaw+=yawd*dt;
   
 ROS_INFO("xd=%f\tyd=%f\tzd=%f\trd=%f\tpd=%f\tyd=%f\tdt=%f\tt=%f\t",xd,yd,zd,rolld,pitchd,yawd,dt,current_time);

   //print to screen
  //ROS_INFO("xd=%f\tyd=%f\tzd=%f\trd=%f\tpd=%f\tyd=%f\tdt=%f\t",x,y,z,roll,pitch,yaw,dt);
  
//ROS_INFO("dt=%f\t",dt);
  //ROS_INFO("time=%f\t",current_time);

   cart_msg.t=current_time;
   cart_msg.x=-z;
   cart_msg.y=y;
   cart_msg.z=-x;
   cart_msg.roll=-roll;
   cart_msg.pitch=-yaw;
   cart_msg.yaw=-pitch;
   cart_msg.xd=-zd;
   cart_msg.yd=yd;
   cart_msg.zd=-xd;
   cart_msg.rolld=-rolld;
   cart_msg.pitchd=-yawd;
   cart_msg.yawd=-pitchd;

   cart_pub.publish(cart_msg);
   }
   
   // moving base mode
   else if (mode==2){
   ROS_INFO("Moving base mode");


   linear_x=scale_lin*msg->axes[0];
   if (linear_x>x_min){
      linear_x=(linear_x-x_min)*x_range/(1-x_min);
      }
   else if (linear_x<-x_min){ 
     linear_x=(linear_x+x_min)*x_range/(1-x_min);
     } 
   else
   {linear_x=0;}


   linear_y=scale_lin*msg->axes[1];
   if ( linear_y>x_min){
       linear_y=( linear_y-x_min)*x_range/(1-x_min);
      }
   else if ( linear_y<-x_min){ 
      linear_y=( linear_y+x_min)*x_range/(1-x_min);
     } 
   else
   { linear_y=0;}

 
  linear_z=msg->axes[2];
   if (linear_z>z_min){
      linear_z=(linear_z-z_min)*z_base_range/(1-z_min); //Joystick commands always scale between 0-1. So this is correct formula
      }
   else if (linear_z<-z_min){ 
     linear_z=(linear_z+z_min)*z_base_range/(1-z_min);
     } 
   else
   {linear_z=0;}


   angular_z=scale_ang*msg->axes[5];
   if (angular_z>x_min){
      angular_z=(angular_z-x_min)*x_range/(1-x_min);
      }
   else if (angular_z<-x_min){ 
     angular_z=(angular_z+x_min)*x_range/(1-x_min);
     } 
   else
   {angular_z=0;}

   //apply acceleration limits

   if (linear_x>vx){
      vx=vx+acc_lin*dt;
      if (vx>=linear_x){
         vx=linear_x;
         };
      }
   else if (linear_x<vx){
      vx=vx-acc_lin*dt;
      if (vx<=linear_x){
         vx=linear_x;
         };
      };

   if (linear_y>vy){
      vy=vy+acc_lin*dt;
      if (vy>=linear_y){
         vy=linear_y;
         };
      }
   else if (linear_y<vy){
      vy=vy-acc_lin*dt;
      if (vy<=linear_y){
         vy=linear_y;
         };
      };

   if (angular_z>vz){
      vz=vz+acc_ang*dt;
      if (vz>=angular_z){
         vz=angular_z;
         };
      }
   else if (angular_z<vz){
      vz=vz-acc_ang*dt;
      if (vz<=angular_z){
         vz=angular_z;
         };
      };

   vel_msg.linear.x=linear_x;
   vel_msg.linear.y=linear_y;
   vel_msg.linear.z=linear_z;
   vel_msg.angular.z=angular_z;

   vel_pub.publish(vel_msg);
   ROS_INFO("x=%f\ty=%f\tz=%f\tz_ang=%f\t",vx,vy,linear_z,vz);
   };


   last_time = ros::WallTime::now().toSec()-init_time;
}



int main(int argc, char** argv){
  ros::init(argc, argv, "spacenav_publisher");

  ros::NodeHandle n;
  ros::Subscriber spacenav_sub = n.subscribe("/spacenav/joy", 1, &SpaceNavCallback);
  cart_pub = n.advertise<amigo_msgs::tip_ref>("cart_coordinates", 1);
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
