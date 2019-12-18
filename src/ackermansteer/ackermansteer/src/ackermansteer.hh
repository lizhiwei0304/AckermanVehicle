/*
*  Copyright 2019 Forrest Edwards
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights 
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
* copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all 
* copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. * IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#ifndef ACKERMANSTEER_SRC_ACKERMANSTEER_HH_
#define ACKERMANSTEER_SRC_ACKERMANSTEER_HH_

#include <map>

// Gazebo
#include<gazebo/common/common.hh>
#include<gazebo/physics/physics.hh>
#include<gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <algorithm>
#include <assert.h>

//ignition
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include<string>
#include<cmath>
#include<vector>
#include <sdf/sdf.hh>



namespace gazebo {
// Wheel order follows cartestion quadrant numbering
// when x axis indicates primary direction of motion

class Joint;
class Entity;

class AckermanSteer : public ModelPlugin {
  enum { FL, FR, RR, RL };
  enum {X, Y, Z};
  enum OdomSource
          {
                ENCODER=0,
                WORLD=1,
          };

 public:
    AckermanSteer();
    ~AckermanSteer();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    //void OnUpdate();
    void Reset();

    //added by gazebo
protected:
    virtual void UpdateChild();
    virtual void FiniChild();

 private:
    //common::Time GazeboTime();
    //added by ackemansteer
    std::vector<double> GetAckAngles(double phi);
    std::vector<double> GetDiffSpeeds(double vel, double phi);



    //added by lizhiwei
    void publishOdometry(double step_time);
    //void getWheelVelocities();
    void publishWheelTF(); /// publishes the wheel tf's
    //void publishSteerTF();
    void publishWheelJointState();
        void UpdateOdometryEncoder();

    double wheel_separation_;
    double wheelbase_;
    double wheel_diameter_;
    double wheel_accel_;
    double wheel_torque_;
    double wheel_speed_[4];
    double wheel_speed_instr_[4];

    std::string drive_joint_names_[4];
    std::string steer_joint_names_[4];



    //added by gazebo
    GazeboRosPtr gazebo_ros_;
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection_;

    // ROS STUFF
    ros::Publisher odometry_publisher_;
    ros::Subscriber cmd_vel_subscriber_;
    boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
    sensor_msgs::JointState steer_joint_state_,drive_joint_state_;
    ros::Publisher steer_joint_state_publisher_,drive_joint_state_publisher_;
    nav_msgs::Odometry odom_;
    std::string tf_prefix_;
    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster odom_broadcaster;
    boost::mutex lock;

    std::string robot_namespace_;
    std::string command_topic_;
    std::string odometry_topic_;
    std::string odometry_frame_;
    std::string robot_base_frame_;


    bool publish_tf_;
    bool legacy_mode_;

    //Custom Callback Queue
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;
    void QueueThread();

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

    double x_;
    double rot_;
    bool alive_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_update_time_;

    OdomSource odom_source_;
    geometry_msgs::Pose2D pose_encoder_;
    common::Time last_odom_update_;

    //Flags
    bool debug_;
    bool publishWheelTF_;
    bool publishSteerTF_;
    bool publishOdomTF_;
    bool publishWheelJointState_;

    double drive_p_;
    double drive_i_;
    double drive_d_;
    double drive_imax_;
    double drive_imin_;
    double drive_init_velocity_;
    double drive_cmd_max_;
    double steer_p_;
    double steer_i_;
    double steer_d_;
    double steer_imax_;
    double steer_imin_;
    double steer_max_effort_;
    double steer_init_angle_;
    double steer_cmd_max_;

    std::vector<physics::JointPtr> steer_joints_, drive_joints_;
    std::vector<common::PID> steer_PIDs_, drive_PIDs_;
    std::vector<double> steer_target_angles_, drive_target_velocities_;
};
}  // namespace gazebo

#endif  // ACKERMANSTEER_SRC_ACKERMANSTEER_HH_
