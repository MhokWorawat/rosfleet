#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

// Define initial variables for odometry
double X = 0.0;
double Y = 0.0;
double th = 0.0;

double Vx = 0.0;              // Linear velocity in x-direction (m/s)
double steering_angle = 0.0;  // Steering angle (rad)
double wheelbase = 0.517;     // Distance between front and rear axles (m)

void VelocityCallback(const geometry_msgs::Twist& velocity) {
    Vx = velocity.linear.x;
    steering_angle = velocity.angular.z; // Steering angle is provided in angular.z
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;

    // Set up publisher and subscriber
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber velocity_sub = n.subscribe("Velocity", 100, VelocityCallback);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time currentTime, lastTime;
    currentTime = ros::Time::now();
    lastTime = ros::Time::now();

    ros::Rate r(10.0); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();  // check for incoming messages
        currentTime = ros::Time::now();

        // Compute odometry
        double deltaTime = (currentTime - lastTime).toSec();

        // Compute the instantaneous center of rotation (ICR)
        double delta_th = (Vx / wheelbase) * tan(steering_angle) * deltaTime;

        double delta_X = Vx * cos(th + delta_th / 2) * deltaTime;
        double delta_Y = Vx * sin(th + delta_th / 2) * deltaTime;

        X += delta_X;
        Y += delta_Y;
        th += delta_th;

        // Normalize the angle th to be within the range -π to π
        th = atan2(sin(th), cos(th));

        // Create a quaternion from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        // Publish the transform
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = currentTime;
        odom_trans.header.frame_id = "agv03_odom";
        odom_trans.child_frame_id = "agv03_base";

        odom_trans.transform.translation.x = X;
        odom_trans.transform.translation.y = Y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        // Publish the odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = currentTime;
        odom.header.frame_id = "agv03_odom";

        // Set the position
        odom.pose.pose.position.x = X;
        odom.pose.pose.position.y = Y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // Set the velocity
        odom.child_frame_id = "agv03_base";
        odom.twist.twist.linear.x = Vx;
        odom.twist.twist.angular.z = delta_th / deltaTime;

        // Publish the message
        odom_pub.publish(odom);

        lastTime = currentTime;
        r.sleep();
    }
}
