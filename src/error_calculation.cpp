#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <sstream>

double x_est = 0,y_est = 0,fi_est=0, fi_calc = 0;
double x_odom,y_odom,z_odom;
double x_gazebo,y_gazebo,z_gazebo;
double v_lin, v_ang;
double error, error_sum, est_error, est_error_sum;
double time_quantum = double(1)/double(30); //moznaby to zmienic na mierzenie kwantu czasu pomiedzy odebraniem funkcji zamiast zakladanie ze ros faktycznie publikuje z 30 Hz
double x, y;
int mode;
geometry_msgs::Pose gazebo_pose;

void roundFiEst()
{
    fi_est = abs(fi_est);
    if (fi_est > M_PI/4 && fi_est < 3*M_PI/4)
    {
        fi_calc = M_PI/2;
    }
    else if ( fi_est > 3*M_PI/4 && fi_est < 5*M_PI/4)
    {
        fi_calc = M_PI;
    }
    else if ( fi_est > 5*M_PI/4 && fi_est < 7*M_PI/4)
    {
        fi_calc = 3*M_PI/2;
    }
    else if ( fi_est > 7*M_PI/4 && fi_est < 9*M_PI/4)
    {
        fi_calc = 2*M_PI;
    }
    
}
void odoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    x_odom = msg->pose.pose.position.x;
    y_odom = msg->pose.pose.position.y;
    //z_odom = msg->pose.pose.position.z;
    
}


void estCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    v_lin = msg->linear.x;
    v_ang = msg->angular.z;
    x_est = x_est + v_lin*time_quantum*cos(fi_est);
    y_est = y_est + v_lin*time_quantum*sin(fi_est);
    fi_est = fi_est + v_ang*time_quantum;
    // roundFiEst();

}


void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    x_gazebo = msg->pose[1].position.x;
    y_gazebo = msg->pose[1].position.y;
    z_gazebo = msg->pose[1].position.z;
}


float calculate_error(float x, float y, float x_gazebo, float y_gazebo)
{
    float error = sqrt(pow((x - x_gazebo),2) + pow((y - y_gazebo),2));
    return error;
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "error_calculation");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);
    // int mode = 1; // mode = 1 dla bledu bez odometrii; mode = 2 dla bledu z odometria
    n.getParam("lab1/mode", mode);


    ros::Subscriber sub_odom = n.subscribe("/mobile_base_controller/odom", 1000, odoCallback);
    ros::Subscriber sub_pose_est = n.subscribe("/key_vel", 1000, estCallback);
    ros::Subscriber sub_gazebo = n.subscribe("/gazebo/model_states", 1000, gazeboCallback);

    ros::Publisher error_pub = n.advertise<std_msgs::Float64>("/error", 1000);
    ros::Publisher error_sum_pub = n.advertise<std_msgs::Float64>("/error_sum", 1000);

    std_msgs::Float64 error_msg;
    std_msgs::Float64 error_sum_msg;
    while (ros::ok())
    {

        if(mode == 2)
        {
            error = calculate_error(x_odom, y_odom, x_gazebo, y_gazebo);
            std::cout << "odomX: "<< x_odom << " | " << x_gazebo <<" odomY: "<< y_odom <<  " | " << y_gazebo << std::endl;

        }
        else if(mode == 1)
        {
            error = calculate_error(x_est, y_est, x_gazebo, y_gazebo);
            std::cout << "estX: "<< x_est <<" estY: "<< y_est <<" estFi: " << fi_est <<" FiCalc: " << fi_calc <<  " v: " << v_lin << " vr: "<< v_ang<<std::endl;
        }

        if(v_lin != 0 || v_ang != 0)
        {
            error_sum = error_sum + error;
            
            error_msg.data = error;
            error_sum_msg.data = error_sum;
        
            error_pub.publish(error_msg);
            error_sum_pub.publish(error_sum_msg);
        }
        

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
