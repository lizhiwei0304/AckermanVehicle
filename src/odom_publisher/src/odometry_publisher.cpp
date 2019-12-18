/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//#include <boost/thread.hpp>
#include <math.h>

struct OdomFlant {

    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float ax;
    float ay;
    float az;
    float th;
    tf::Quaternion quaternion;
};
struct OdomFlant flant;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

    flant.x = msg->pose.pose.position.x;
    flant.y = msg->pose.pose.position.y;
    flant.z = msg->pose.pose.position.z;

    flant.th = tf::getYaw(msg->pose.pose.orientation);

    flant.vx = msg->twist.twist.linear.x;
    flant.vy = msg->twist.twist.linear.y;
    flant.vz = msg->twist.twist.linear.z;

    flant.ax = msg->twist.twist.angular.x;
    flant.ay = msg->twist.twist.angular.y;
    flant.az = msg->twist.twist.angular.z;

}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odomRviz", 50);
  ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/groundtruth/state",100,OdomCallback);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time;
  current_time = ros::Time::now();
  ros::Rate r(50);
  while(n.ok()){
    current_time = ros::Time::now();

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(flant.th);
    /*odom_quat.x = flant.th_x;
    odom_quat.y = flant.th_y;
    odom_quat.z = flant.th_z;
    odom_quat.w = flant.th_w;
    std::cout <<"I heard %lf x"<< odom_quat.x<<std::endl;
    std::cout <<"I heard %lf y"<< odom_quat.y<<std::endl;
    std::cout <<"I heard %lf w"<< odom_quat.w<<std::endl;
    std::cout <<"I heard %lf z"<< odom_quat.z<<std::endl;*/
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = flant.x;
    odom_trans.transform.translation.y = flant.y;
    odom_trans.transform.translation.z = flant.z;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = flant.x;
    odom.pose.pose.position.y = flant.y;
    odom.pose.pose.position.z = flant.z;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = flant.vx;
    odom.twist.twist.linear.y = flant.vy;
    odom.twist.twist.linear.z = flant.vz;

    odom.twist.twist.angular.x = flant.ax;
    odom.twist.twist.angular.y = flant.ay;
    odom.twist.twist.angular.z = flant.az;

    //publish the message
    odom_pub.publish(odom);

    ros::spinOnce();
    r.sleep();
  }
}
