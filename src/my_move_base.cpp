#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <sstream>
#include "tf/transform_listener.h"
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "global_planner/planner_core.h"
#include "geometry_msgs/PoseStamped.h"
#include <gazebo_msgs/ModelStates.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <rotate_recovery/rotate_recovery.h>
#include <iostream>
#include <string>  

double x_gazebo,y_gazebo,z_gazebo;
float angX, angY, angZ, angW, angFi;
geometry_msgs::PoseStamped startPosition, finishPosition;
bool new_goal = false;
std::vector<geometry_msgs::PoseStamped> globalPlan;
ros::Time stop_time;
geometry_msgs::Twist planned_vel;
bool moving_on = false;
bool is_vel_planned = false;
bool wait_for_rec = false;
float x,y;
std_msgs::String info_msg;
int k = 0;


// void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
// {
//     int a = 2;
//     x_gazebo = msg->pose[a].position.x;
//     y_gazebo = msg->pose[a].position.y;
//     z_gazebo = msg->pose[a].position.z;
//     startPosition.pose.position.x = x_gazebo;
//     startPosition.pose.position.y = y_gazebo;
//     startPosition.pose.position.z = z_gazebo;

//     startPosition.pose.orientation.x = msg->pose[a].orientation.x;
//     startPosition.pose.orientation.y = msg->pose[a].orientation.y;
//     startPosition.pose.orientation.z = msg->pose[a].orientation.z;
//     startPosition.pose.orientation.w = msg->pose[a].orientation.w;

//     startPosition.header.frame_id = "map";
// }

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  finishPosition = *msg;
  new_goal = true;
  finishPosition.header.frame_id = "map";
  x = finishPosition.pose.position.x;
  y = finishPosition.pose.position.y;
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_move_base");
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Publisher nav_info_publisher = n.advertise<std_msgs::String>("/nav_info", 1000);

  ros::Rate loop_rate(30);

  ros::Subscriber sub = n.subscribe("/move_base_simple/goal", 1000, goalCallback);
  // ros::Subscriber sub_gazebo = n.subscribe("/gazebo/model_states", 1000, gazeboCallback);
  geometry_msgs::Twist msg;

  tf2_ros::Buffer tf_buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(tf_buffer);

  // global costmap
  costmap_2d::Costmap2DROS my_global_costmap("global_costmap", tf_buffer);

  // local costmap
  costmap_2d::Costmap2DROS my_local_costmap("local_costmap", tf_buffer);

  // global planner
  global_planner::GlobalPlanner my_global_planner("global_planner", my_global_costmap.getCostmap(), "map");

  // local planner
  base_local_planner::TrajectoryPlannerROS my_local_planner;
  my_local_planner.initialize("local_planner", &tf_buffer, &my_local_costmap);

  //recovery behavior
  rotate_recovery::RotateRecovery rr;
  rr.initialize("rotate_recovery", &tf_buffer, &my_global_costmap, &my_local_costmap);
  

  while (ros::ok())
  {
    my_global_costmap.getRobotPose(startPosition);
    bool valid_plan;
    if(new_goal)
      {
        ROS_INFO("-");
        ROS_INFO("-");
        ROS_INFO("New goal has been set at x = %f, y = %f", x,y );
        
        char x_array[10];
        sprintf(x_array, "%f", x);
        char y_array[10];
        sprintf(y_array, "%f", y);

        std::string info1("New goal has been set at x = ");
        std::string info2(" y = ");
        info1.append(x_array);
        info1.append(info2);
        info1.append(y_array);
        info_msg.data = info1;
        nav_info_publisher.publish(info_msg);

        valid_plan = my_global_planner.makePlan(startPosition, finishPosition, globalPlan);
        if(valid_plan){
          ROS_INFO("Global plan has been created");
          info_msg.data = "Global plan has been created";
          nav_info_publisher.publish(info_msg);

          my_global_planner.publishPlan(globalPlan);
          my_local_planner.setPlan(globalPlan);
        }else{
          ROS_INFO("Tiago cannot reach given point");
          info_msg.data = "Tiago cannot reach given point";
          nav_info_publisher.publish(info_msg);
        }
        
        new_goal = false; 
        moving_on = true;
      }
    

    if(moving_on){
      is_vel_planned = my_local_planner.computeVelocityCommands(planned_vel);  
      if (is_vel_planned){
        vel_pub.publish(planned_vel);
        wait_for_rec = false;
        if(k%90 == 0){
          info_msg.data = "Tiago is moving";
          nav_info_publisher.publish(info_msg);
        }
      }
      else if(wait_for_rec == false) {
        stop_time = ros::Time::now();
        wait_for_rec = true;
        ROS_INFO("If path not find in 1 sec Recovery will start");
        info_msg.data = "If path not find in 1 sec Recovery will start";
        nav_info_publisher.publish(info_msg);
      }
      if(wait_for_rec == true && ros::Time::now()-stop_time>ros::Duration(1.0))
      {
        ROS_INFO("Recovery started");
        info_msg.data = "Recovery started";
        nav_info_publisher.publish(info_msg);
        rr.runBehavior();
      }
      

    
      if(my_local_planner.isGoalReached()){
        ROS_INFO("Goal is reached");
        info_msg.data = "Goal is Reached";
        nav_info_publisher.publish(info_msg);
        moving_on = false;
      }
    } 

    k = k+1;
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
