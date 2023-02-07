#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <sstream>

float x,y,z;
float angX, angY, angZ, angW, angFi;


void odoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
 x = msg->pose.pose.position.x;
 y = msg->pose.pose.position.y;
 z = msg->pose.pose.position.z;

angX = msg->pose.pose.orientation.x;
angY = msg->pose.pose.orientation.y;
angZ = msg->pose.pose.orientation.z;
angW = msg->pose.pose.orientation.w;

angFi = atan2(2*angW*angZ,1-2*angZ*angZ);

//liczenie kata TODO

}

int main(int argc, char **argv)
{

  float vel;
  float vr;
  float d;
  float startX = 0;
  float startY = 0;
  float startZ = 0;
  int mode;
  double duration = 0;
  ros::Time start;

  ros::init(argc, argv, "lab1");
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/key_vel", 1000);
  ros::Rate loop_rate(30);

  ros::Subscriber sub = n.subscribe("/mobile_base_controller/odom", 1000, odoCallback);
  geometry_msgs::Twist msg;

  n.getParam("lab1/mode", mode);
  n.getParam("lab1/vel", vel);
  n.getParam("lab1/vr", vr);
  n.getParam("lab1/d", d);
  

  while (ros::ok())
  {
    vel = 2.4*vel;
    vr = 2.4*vr;
    msg.linear.y = 0;
    msg.linear.z = 0;

    msg.angular.x = 0;
    msg.angular.y = 0;
    if (mode == 3 || mode == 4)
    {
      vr = -1 * vr;
    }
    ros::Duration(4).sleep();

    //ros:Time finish = ros::Time::now();
    if (mode == 1 || mode == 3)
    {
      for(int i=0;i<4;i++)
      {
        ros::Duration(1).sleep();
        start = ros::Time::now();
        while (ros::Time::now() - start < ros::Duration((d/vel)) )
        {
          msg.linear.x = vel;
          msg.angular.z = 0;
          vel_pub.publish(msg);
          ros::spinOnce();
          loop_rate.sleep();
          //finish = clock();
          //duration = double(finish-start)/CLOCKS_PER_SEC;  
        }

        ros::Duration(1).sleep();
        start = ros::Time::now();
        while (ros::Time::now() - start < ros::Duration((1.570796/abs(vr))))
        {
          msg.linear.x = 0;
          msg.angular.z = vr;
          vel_pub.publish(msg);
          ros::spinOnce();
          loop_rate.sleep();
          //finish = clock();
          //duration = double(finish-start)/CLOCKS_PER_SEC;  
        }

      }
      msg.angular.z = 0;
      vel_pub.publish(msg);
      mode = 0;
      
    }
    else if(mode == 2 || mode == 4)
    {

      for(int i=0;i<4;i++)
      {
        startX = x;
        startY = y;
        ros::Duration(1).sleep();
        while(sqrt(pow((x-startX),2)+pow((y-startY),2))<=d)
        {
          msg.linear.x = vel;      
          msg.angular.z = 0;
          vel_pub.publish(msg);
          ros::spinOnce();
          loop_rate.sleep();
        }
        msg.linear.x = vel; 
        vel_pub.publish(msg);
        ros::Duration(1).sleep();
        float startANG = angFi;
        while (abs(angFi-startANG)<=1.570796)//(ros::Time::now() - start < ros::Duration((1.570796/vr)))
        {
          msg.linear.x = 0;
          msg.angular.z = vr;
          vel_pub.publish(msg);
          ros::spinOnce();
          loop_rate.sleep();
        }
        msg.angular.z = 0;
        vel_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
      }
      /*
      ros::Duration(1).sleep();
      startX = x;
      while(x-startX<=d)
      {
        vel_pub.publish(msg);
        msg.linear.x = vel;
        msg.angular.z = 0;
        ros::spinOnce();
        loop_rate.sleep();
      }
      msg.linear.x = 0
      ros::Duration(1).sleep();
      //start = ros::Time::now();
      float startANG = angFi;
      while (angFi-startANG<=1.570796)//(ros::Time::now() - start < ros::Duration((1.570796/vr)))
      {
        msg.linear.x = 0;
        msg.angular.z = vr;
        vel_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
      }
      msg.angular.z = 0;
      ros::Duration(1).sleep();
      */
      mode = 0;
    }
    

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
