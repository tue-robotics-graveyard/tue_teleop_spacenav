#include <ros/ros.h>
#include <joy/Joy.h>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <amigo_msgs/tip_ref.h>
#include <tf/transform_broadcaster.h>

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
double x_min=0.0014;
double init_time;
double scale=0.01;
ros::Publisher cart_pub;
amigo_msgs::tip_ref cart_msg;


void SpaceNavCallback(const joy::Joy::ConstPtr& msg)
{  current_time = ros::Time::now().toSec()-init_time;
   dt=current_time-last_time;
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
   if (xd>-x_min && xd<x_min)
      xd=0;
   else
   {xd-=0;}

   yd=-1*scale*(msg->axes[1]);
   if (yd>-x_min && yd<x_min)
      yd=0;
   else
   {yd-=0;}

   zd=-1*scale*(msg->axes[2]);
   if (zd>-x_min && zd<x_min)
      zd=0;
   else
   {zd-=0;}

   rolld=-1*scale*(msg->axes[3]);
   if (rolld>-x_min && rolld<x_min)
      rolld=0;
   else
   {rolld-=0;}

   pitchd=-1*scale*(msg->axes[4]);
   if (pitchd>-x_min && pitchd<x_min)
      pitchd=0;
   else
   {pitchd-=0;}

   yawd=-1*scale*(msg->axes[5]);
   if (yawd>-x_min && yawd<x_min)
      yawd=0;
   else
   {yawd-=0;}

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
   cart_msg.x=x;
   cart_msg.y=y;
   cart_msg.z=z;
   cart_msg.roll=roll;
   cart_msg.pitch=pitch;
   cart_msg.yaw=yaw;
   cart_msg.xd=xd;
   cart_msg.yd=yd;
   cart_msg.zd=zd;
   cart_msg.rolld=rolld;
   cart_msg.pitchd=pitchd;
   cart_msg.yawd=yawd;

   cart_pub.publish(cart_msg);

   
   last_time = ros::Time::now().toSec()-init_time;
}



int main(int argc, char** argv){
  ros::init(argc, argv, "spacenav_publisher");

  ros::NodeHandle n;
  ros::Subscriber spacenav_sub = n.subscribe("/spacenav/joy", 1, &SpaceNavCallback);
  ros::Publisher odom_pub2 = n.advertise<nav_msgs::Odometry>("/odom", 1);
  cart_pub = n.advertise<amigo_msgs::tip_ref>("cart_coordinates", 1);
  ros::Rate rate(1000);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time2;
  init_time = ros::Time::now().toSec();
  while (n.ok()){





    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
    current_time2 = ros::Time::now();

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time2;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = z;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time2;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = xd;
    odom.twist.twist.linear.y = yd;
    odom.twist.twist.angular.z = zd;

    //publish the message  (misschien staan x en y nog omgewisseld)
    odom_pub2.publish(odom);












  ros::spinOnce();
  rate.sleep();
  }
  return 1;

}
