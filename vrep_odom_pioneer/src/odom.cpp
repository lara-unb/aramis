#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped my_pose;
geometry_msgs::TwistStamped my_twist;

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& vrep_twist)
{
  my_twist.twist = vrep_twist->twist;
  my_twist.header = vrep_twist->header;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& vrep_pose)
{
  my_pose.pose = vrep_pose->pose;
  my_pose.header = vrep_pose->header;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber pose_sub = n.subscribe("pose_pioneer", 100, poseCallback);
  ros::Subscriber twist_sub = n.subscribe("twist_pioneer", 100, twistCallback);

  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(30);
  while(n.ok())
  {

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    // double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;

    // x += delta_x;
    // y += delta_y;

    x = my_pose.pose.position.x;
    y = my_pose.pose.position.y;

    vx = my_twist.twist.linear.x;
    vy = my_twist.twist.linear.y;
    vth = my_twist.twist.angular.z;

    double delta_th = vth * dt;
    // th = (vx * sin(th) + vy * cos(th)) * dt;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = my_pose.pose.orientation;

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    // odom.pose.covariance = my_pose.covariance;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    // odom.twist.covariance = twist.covariance;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
