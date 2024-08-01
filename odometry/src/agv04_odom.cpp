#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"

double X = 0;
double Y = 0;
double th = 0;

double Vx = 0;   // .M/s
double Vy = 0;   // .M/s
double Vth = 0;  // .M/s

void VelocityX_Callback(const std_msgs::Float32& LinearX){
  Vx = LinearX.data;
}
void VelocityY_Callback(const std_msgs::Float32& LinearY){
  Vy = LinearY.data;
}
void VelocityZ_Callback(const std_msgs::Float32& AngularZ){
  Vth = AngularZ.data;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;

  std::string odom_frame_id = "agv04_odom";
  std::string base_frame_id = "agv04_base";

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  ros::Subscriber subVx = n.subscribe("LinearX", 100, VelocityX_Callback);
  ros::Subscriber subVy = n.subscribe("LinearY", 100, VelocityY_Callback);
  ros::Subscriber subVth = n.subscribe("AngularZ", 100, VelocityZ_Callback);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time currentTime, lastTime;
  currentTime = ros::Time::now();
  lastTime = ros::Time::now();

  ros::Rate r(10.0);
  while(ros::ok()){
    ros::spinOnce();                         // check for incoming messages
    currentTime = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double deltaTime = (currentTime - lastTime).toSec();
    double delta_X = (Vx * cos(th) - Vy * sin(th)) * deltaTime;
    double delta_Y = (Vx * sin(th) + Vy * cos(th)) * deltaTime;
    double delta_th = Vth * deltaTime;

    X += delta_X;
    Y += delta_Y;
    th += delta_th;

    // Normalize the angle th to be within the range -π to π
    th = atan2(sin(th), cos(th));

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = currentTime;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;

    odom_trans.transform.translation.x = X;
    odom_trans.transform.translation.y = Y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = odom_frame_id;

    //set the position
    odom.pose.pose.position.x = X;
    odom.pose.pose.position.y = Y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = base_frame_id;
    odom.twist.twist.linear.x = Vx;
    odom.twist.twist.linear.y = Vy;
    odom.twist.twist.angular.z = Vth;

    //publish the message
    odom_pub.publish(odom);
    lastTime = currentTime;

    r.sleep();
  }
}
