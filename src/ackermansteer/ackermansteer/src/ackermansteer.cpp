#include "ackermansteer.hh"

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

namespace gazebo {

  // Constructor
  AckermanSteer::AckermanSteer() {}

  // Destructor
  AckermanSteer::~AckermanSteer() {
	FiniChild();
}

  // Required Load method:
  //需要修改车辆的参数
void AckermanSteer::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
      // Store the pointer to the Model
      this->model = _parent;
      // Create a new GazeboRos instance
      gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "AckermanSteer"));

      // Check to see if ros is initialized.Prints ROS_FATAL and return if not.
      gazebo_ros_->isInitialized();

      // Function template call to getParameter. Retrieves params passed in _sdf
      gazebo_ros_->getParameterBoolean
              (debug_, "debug", false);
      gazebo_ros_->getParameter<std::string>
              (drive_joint_names_[FL], "FL_driveJoint", "front_left_wheel_bearing");
      gazebo_ros_->getParameter<std::string>
              (drive_joint_names_[FR], "FR_driveJoint", "front_right_wheel_bearing");
      gazebo_ros_->getParameter<std::string>
              (drive_joint_names_[RL], "RL_driveJoint", "rear_left_wheel_bearing");
      gazebo_ros_->getParameter<std::string>
              (drive_joint_names_[RR], "RR_driveJoint", "rear_right_wheel_bearing");
      gazebo_ros_->getParameter<std::string>
              (steer_joint_names_[FL], "FL_steerJoint", "front_left_steer_bearing");
      gazebo_ros_->getParameter<std::string>
              (steer_joint_names_[FR], "FR_steerJoint", "front_right_steer_bearing");
      gazebo_ros_->getParameter<std::string>
              (steer_joint_names_[RL], "RL_steerJoint", "rear_left_steer_bearing");
      gazebo_ros_->getParameter<std::string>
              (steer_joint_names_[RR], "RR_steerJoint", "rear_right_steer_bearing");
      gazebo_ros_->getParameter<std::string>
              (command_topic_, "commandTopic", "cmd_vel");
      gazebo_ros_->getParameter<std::string>
              (odometry_topic_, "odometryTopic", "odom");
      gazebo_ros_->getParameter<std::string>
              (odometry_frame_, "odometryFrame", "odom");
      gazebo_ros_->getParameter<std::string>
              (robot_base_frame_, "robotBaseFrame", "base_link");
      gazebo_ros_->getParameterBoolean
              (publishWheelTF_, "publishWheelTF", false);
      gazebo_ros_->getParameterBoolean
              (publishSteerTF_, "publishSteerTF", false);
      gazebo_ros_->getParameterBoolean
              (publishOdomTF_, "publishOdomTF", false);
      gazebo_ros_->getParameterBoolean
              (publishWheelJointState_, "publishWheelJointState", false);
      gazebo_ros_->getParameterBoolean(legacy_mode_, "legacyMode", true);

      if (!_sdf->HasElement("legacyMode")) {
          ROS_ERROR_NAMED("AckermanSteer", "GazeboRoscakemanDrive Plugin missing <legacyMode>, defaults to true\n"
                                           "This setting assumes you have a old package, where the right and left wheel are changed to fix a former code issue\n"
                                           "To get rid of this error just set <legacyMode> to false if you just created a new package.\n"
                                           "To fix an old package you have to exchange left wheel by the right wheel.\n"
                                           "If you do not want to fix this issue in an old package or your z axis points down instead of the ROS standard defined in REP 103\n"
                                           "just set <legacyMode> to true.\n");
      }

      gazebo_ros_->getParameter<double>
              (wheel_separation_, "wheelSeparation", 1.535);
      gazebo_ros_->getParameter<double>
              (wheelbase_, "wheelbase", 3.271);
      gazebo_ros_->getParameter<double>
              (wheel_diameter_, "wheelDiameter", 0.8693);
      gazebo_ros_->getParameter<double>
              (wheel_accel_, "wheelAcceleration", 0.0);
      gazebo_ros_->getParameter<double>
              (wheel_torque_, "wheelTorque", 5000000.0);
      gazebo_ros_->getParameter<double>
              (update_rate_, "updateRate", 50.0);
      gazebo_ros_->getParameter<double>
              (steer_p_, "steer_p", 100.0);
      gazebo_ros_->getParameter<double>
              (steer_i_, "steer_i", 0.0);
      gazebo_ros_->getParameter<double>
              (steer_d_, "steer_d", 0.0);
      gazebo_ros_->getParameter<double>
              (steer_imax_, "steer_imax", 1.0);
      gazebo_ros_->getParameter<double>
              (steer_imin_, "steer_imin", 1.0);
      gazebo_ros_->getParameter<double>
              (steer_cmd_max_, "steer_max_effort", 200000.0);
      gazebo_ros_->getParameter<double>
              (steer_init_angle_, "steer_init_angle", 0.0);
      gazebo_ros_->getParameter<double>
              (drive_p_, "drive_p", 100.0);
      gazebo_ros_->getParameter<double>
              (drive_i_, "drive_i", 0.0);
      gazebo_ros_->getParameter<double>
              (drive_d_, "drive_d", 0.0);
      gazebo_ros_->getParameter<double>
              (drive_imax_, "drive_imax", 10.0);
      gazebo_ros_->getParameter<double>
              (drive_imin_, "drive_imin", 10.0);
      gazebo_ros_->getParameter<double>
              (drive_cmd_max_, "drive_max_effort", 1.0);
      gazebo_ros_->getParameter<double>
              (drive_init_velocity_, "drive_init_velocity", 0.0);


      // create dictionary with string keys, and OdomSource values.
      // Can be either ENCODER(0) or WORLD(1)
      std::map <std::string, OdomSource> odomOptions;
      odomOptions["encoder"] = ENCODER;
      odomOptions["world"] = WORLD;
      gazebo_ros_->getParameter<OdomSource>
              (odom_source_, "odometrySource", odomOptions, WORLD);

      // Joints & PID's
      steer_joints_.resize(4);
      drive_joints_.resize(4);
      steer_PIDs_.resize(4);
      drive_PIDs_.resize(4);
      for (int i = 0; i < 4; i++) {
          steer_joints_[i] = model->GetJoint(steer_joint_names_[i]);
          drive_joints_[i] = model->GetJoint(drive_joint_names_[i]);
          steer_PIDs_[i].Init(steer_p_, steer_i_, steer_d_, steer_imax_,
                              steer_imin_, steer_cmd_max_, -steer_cmd_max_);
          drive_PIDs_[i].Init(drive_p_, drive_i_, drive_d_, drive_imax_,
                              drive_imin_, drive_cmd_max_, -drive_cmd_max_);
          switch (i) {
              case FL:
              case FR:
                  steer_target_angles_.push_back(steer_init_angle_);
                  drive_target_velocities_.push_back(0.0);
                  break;
              case RL:
              case RR:
                  steer_target_angles_.push_back(0.0);
                  drive_target_velocities_.push_back(drive_init_velocity_);
          }
      }

      this->publish_tf_ = true;
      if (!_sdf->HasElement("publishTf")) {
          ROS_WARN_NAMED("AckermanSteer",
                         "GazeboRosackmansteerDrive Plugin (ns = %s) missing <publishTf>, defaults to %d",
                         this->robot_namespace_.c_str(), this->publish_tf_);
      } else {
          this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
      }

      // Initialize update Rate logic
      if (update_rate_ > 0.0) {
          update_period_ = 1.0 / update_rate_;
      } else {
          update_period_ = 0.0;
      }

#if GAZEBO_MAJOR_VERSION >= 8
      last_update_time_ = model->GetWorld()->SimTime();
#else
      last_update_time_ = model->GetWorld()->GetSimTime();
#endif

      // Initialize velocity stuff
      wheel_speed_[FL] = 0;
      wheel_speed_[FR] = 0;
      wheel_speed_[RL] = 0;
      wheel_speed_[RR] = 0;

      // Initialize velocity support stuff
      wheel_speed_instr_[FL] = 0;
      wheel_speed_instr_[FR] = 0;
      wheel_speed_instr_[RL] = 0;
      wheel_speed_instr_[RR] = 0;

      x_ = 0;
      rot_ = 0;
      alive_ = true;

      if (this->publishWheelJointState_) {
          steer_joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>(
                  "steer_joint_states",
                  1000);
          drive_joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>(
                  "drive_joint_states",
                  1000);
          ROS_INFO_NAMED("AckermanSteer", "%s: Advertise joint_states", gazebo_ros_->info());
      }

      transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

      // ROS PUB-SUB
      ROS_INFO_NAMED("AckermanSteer", "%s: Try to subuscribe to %s",
                     gazebo_ros_->info(), command_topic_.c_str());

      ros::SubscribeOptions so =
              ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                                                                  boost::bind(&AckermanSteer::cmdVelCallback,
                                                                              this,
                                                                              _1),
                                                                  ros::VoidPtr(), &queue_);

      cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
      ROS_INFO_NAMED("AckermanSteer", "%s: Subscribe to %s",
                     gazebo_ros_->info(), command_topic_.c_str());

      if (this->publish_tf_) {
          odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
          ROS_INFO_NAMED("AckermanSteer", "%s: Advertise odom on %s ", gazebo_ros_->info(),
                         odometry_topic_.c_str());
      }

      this->callback_queue_thread_ = boost::thread(boost::bind(&AckermanSteer::QueueThread, this));

      //last_update_time_ = this->GazeboTime();
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
              std::bind(&AckermanSteer::UpdateChild, this));

  }

void AckermanSteer::Reset(){
#if GAZEBO_MAJOR_VERSION >= 8
  last_update_time_ = model->GetWorld()->SimTime();
#else
  last_update_time_ = model->GetWorld()->GetSimTime();
#endif
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  x_ = 0;
  rot_ = 0;
    for (int i = 0; i < 4; i++) {

      steer_joints_[i]->SetParam ( "fmax", 0, wheel_torque_ );
      drive_joints_[i]->SetParam ( "fmax", 0, wheel_torque_ );
    
 }

}

void AckermanSteer::publishWheelJointState()
{
    ros::Time current_time = ros::Time::now();

    //steer_joint_state
    steer_joint_state_.header.stamp = current_time;
    steer_joint_state_.name.resize ( steer_joints_.size() );
    steer_joint_state_.position.resize ( steer_joints_.size() );

    for ( int i = 0; i < 4; i++ ) {
        physics::JointPtr steer_joint = steer_joints_[i];

#if GAZEBO_MAJOR_VERSION >= 8
        double position = steer_joint->Position ( 0 );
#else
        double position = steer_joint->GetAngle ( 0 ).Radian();
#endif
        steer_joint_state_.name[i] = steer_joint->GetName();
        steer_joint_state_.position[i] = position;
    }
    steer_joint_state_publisher_.publish ( steer_joint_state_ );
    
    //drive_joint_state
    drive_joint_state_.header.stamp = current_time;
    drive_joint_state_.name.resize ( drive_joints_.size() );
    drive_joint_state_.position.resize ( drive_joints_.size() );

    for ( int i = 0; i < 4; i++ ) {
        physics::JointPtr drive_joint = drive_joints_[i];

#if GAZEBO_MAJOR_VERSION >= 8
        double position = drive_joint->Position ( 0 );
#else
        double position = drive_joint->GetAngle ( 0 ).Radian();
#endif
        drive_joint_state_.name[i] = drive_joint->GetName();
        drive_joint_state_.position[i] = position;
    }
    drive_joint_state_publisher_.publish ( drive_joint_state_ );
}

/*void AckermanSteer::publishSteerTF()
{
    ros::Time current_time = ros::Time::now();
    for ( int i = 0; i < 4; i++ ) {

        std::string steer_frame = gazebo_ros_->resolveTF(steer_joints_[i]->GetChild()->GetName ());
        std::string steer_parent_frame = gazebo_ros_->resolveTF(steer_joints_[i]->GetParent()->GetName ());

#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d poseSteer = steer_joints_[i]->GetChild()->RelativePose();
#else
        ignition::math::Pose3d poseSteer = steer_joints_[i]->GetChild()->GetRelativePose().Ign();
#endif

        tf::Quaternion qts ( poseSteer.Rot().X(), poseSteer.Rot().Y(), poseSteer.Rot().Z(), poseSteer.Rot().W() );
        tf::Vector3 vts ( poseSteer.Pos().X(), poseSteer.Pos().Y(), poseSteer.Pos().Z() );

        tf::Transform tfSteer ( qts, vts );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( tfSteer, current_time, steer_parent_frame, steer_frame ) );
    }
}*/

void AckermanSteer::publishWheelTF()
{
    ros::Time current_time = ros::Time::now();
    for ( int i = 0; i < 4; i++ ) {

        std::string wheel_frame = gazebo_ros_->resolveTF(drive_joints_[i]->GetChild()->GetName ());
        std::string wheel_parent_frame = gazebo_ros_->resolveTF(drive_joints_[i]->GetParent()->GetName ());

#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d poseWheel = drive_joints_[i]->GetChild()->RelativePose();
#else
        ignition::math::Pose3d poseWheel = drive_joints_[i]->GetChild()->GetRelativePose().Ign();
#endif

        tf::Quaternion qt ( poseWheel.Rot().X(), poseWheel.Rot().Y(), poseWheel.Rot().Z(), poseWheel.Rot().W() );
        tf::Vector3 vt ( poseWheel.Pos().X(), poseWheel.Pos().Y(), poseWheel.Pos().Z() );

        tf::Transform tfWheel ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );
    }
}

void AckermanSteer::UpdateChild() {

/*    for ( int i = 0; i < 4; i++ ) {
      if ( fabs(wheel_torque_ -drive_joints_[i]->GetParam ( "fmax", 0 )) > 1e-6 ) {
        drive_joints_[i]->SetParam ( "fmax", 0, wheel_torque_ );
      }
    }

    for ( int i = 0; i < 4; i++ ) {
      if ( fabs(wheel_torque_ -steer_joints_[i]->GetParam ( "fmax", 0 )) > 1e-6 ) {
        steer_joints_[i]->SetParam ( "fmax", 0, wheel_torque_ );
      }
    }*/

#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = model->GetWorld()->SimTime();
#else
    common::Time current_time = model->GetWorld()->GetSimTime();
#endif

    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
    if (seconds_since_last_update > update_period_) {
        if (this->publish_tf_) publishOdometry ( seconds_since_last_update );
        if ( publishWheelTF_ ) publishWheelTF();
        //if ( publishSteerTF_ ) publishSteerTF();
        if ( publishWheelJointState_ ) publishWheelJointState();

      double steer_ang_curr, steer_error, steer_cmd_effort;
      double drive_vel_curr, drive_error, drive_cmd_effort;
      std::vector<double> ack_steer_angles = GetAckAngles(rot_);
      std::vector<double> ack_drive_velocities = GetDiffSpeeds(x_, rot_);
      steer_target_angles_[FR] = ack_steer_angles[FR];
      steer_target_angles_[FL] = ack_steer_angles[FL];
      steer_target_angles_[RR] = 0.0;
      steer_target_angles_[RL] = 0.0;
      drive_target_velocities_[FR] = 0.0;
      drive_target_velocities_[FL] = 0.0;
      drive_target_velocities_[RR] = ack_drive_velocities[RR];
      drive_target_velocities_[RL] = ack_drive_velocities[RL];

      for (int i = 0; i < 4; i++) {
        switch (i) {
          case FL:
          case FR:
            steer_ang_curr = steer_joints_[i]->GetAngle(X).Radian();
            steer_error = steer_ang_curr - steer_target_angles_[i];
            steer_cmd_effort = steer_PIDs_[i].Update(steer_error, seconds_since_last_update);
            steer_joints_[i]->SetForce(X, steer_cmd_effort);
            break;
          case RL:
          case RR:
            drive_vel_curr = drive_joints_[i]->GetVelocity(Z) *
               wheel_diameter_/2.0;
            drive_error = drive_vel_curr - drive_target_velocities_[i];
            drive_cmd_effort = drive_PIDs_[i].Update(drive_error, seconds_since_last_update);
            drive_joints_[i]->SetForce(Z, drive_cmd_effort);
        }
        if (debug_) {
          double _pe, _ie, _de;
          double pGain = steer_PIDs_[i].GetPGain();
          steer_PIDs_[i].GetErrors(_pe, _ie, _de);
          ROS_INFO("Steer Joints %i", i);
          ROS_INFO("\tCurrent angle: %f \n", steer_ang_curr);
          ROS_INFO("\tTarget angle: %f \n", steer_target_angles_[i]);
          ROS_INFO("\tAngle Error: %f \n", steer_error);
          ROS_INFO("\tEffort: %f \n", steer_cmd_effort);
          ROS_INFO("\tP Gain: %f\n", pGain);
          ROS_INFO("\tP error: %f ", _pe);
          ROS_INFO("\tI error: %f ", _ie);
          ROS_INFO("\tD error: %f ", _de);
        }
        if (debug_) {
          double _pe, _ie, _de;
          double pGain = drive_PIDs_[i].GetPGain();
          drive_PIDs_[i].GetErrors(_pe, _ie, _de);
          ROS_INFO("Drive Joint %i", i);
          ROS_INFO("\tCurrent Vel: %f \n", drive_vel_curr);
          ROS_INFO("\tTarget Vel: %f \n", drive_target_velocities_[i]);
          ROS_INFO("\tVel Error: %f \n", drive_error);
          ROS_INFO("\tEffort: %f \n", drive_cmd_effort);
          ROS_INFO("\tP Gain: %f\n", pGain);
          ROS_INFO("\tP error: %f ", _pe);
          ROS_INFO("\tI error: %f ", _ie);
          ROS_INFO("\tD error: %f ", _de);
        }
      }
      last_update_time_ = current_time;
    }
  }

void AckermanSteer::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

  //计算阿克曼转向的转角，不需要修改
std::vector<double> AckermanSteer::GetAckAngles(double phi) {
    std::vector<double> phi_angles;
    double numerator = 2.0 * wheelbase_ * sin(phi);
    phi_angles.assign(4, 0.0);
    phi_angles[FL] = atan2(numerator,
       (2.0*wheelbase_*cos(phi) - wheel_separation_*sin(phi)) );
    phi_angles[FR] = atan2(numerator,
       (2.0*wheelbase_*cos(phi) + wheel_separation_*sin(phi)) );
    return phi_angles;
  }

  //计算后轮的轮速
std::vector<double> AckermanSteer::GetDiffSpeeds(double vel, double phi) {
    std::vector<double> wheel_speeds;
    wheel_speeds.assign(4, 0.0);
    wheel_speeds[RL] = vel * (1.0 - (wheel_separation_ * tan(phi) ) /
       (2.0 * wheelbase_) );
    wheel_speeds[RR] = vel * (1.0 + (wheel_separation_ * tan(phi) ) /
       (2.0 * wheelbase_) );
    return wheel_speeds;
  }

  // Called by the world update start event
  //更新车辆的状态，实现前轮转向，后轮驱动
  //不需要修改
void AckermanSteer::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
      boost::mutex::scoped_lock scoped_lock(lock);
      x_ = cmd_msg->linear.x;
      rot_ = cmd_msg->angular.z;
  }

  //设置进程不需要修改
void AckermanSteer::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && gazebo_ros_->node()->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

void AckermanSteer::UpdateOdometryEncoder()
{
    double vl = drive_joints_[RL]->GetVelocity ( 0 );
    double vr = drive_joints_[RR]->GetVelocity ( 0 );
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = model->GetWorld()->SimTime();
#else
    common::Time current_time = model->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

    double b = wheel_separation_;

    // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    double sl = vl * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double sr = vr * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double ssum = sl + sr;

    double sdiff;
    if(legacy_mode_)
    {
      sdiff = sl - sr;
    }
    else
    {

      sdiff = sr - sl;
    }

    double dx = ( ssum ) /2.0 * cos ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
    double dy = ( ssum ) /2.0 * sin ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
    double dtheta = ( sdiff ) /b;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    double w = dtheta/seconds_since_last_update;
    double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

    tf::Quaternion qt;
    tf::Vector3 vt;
    qt.setRPY ( 0,0,pose_encoder_.theta );
    vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    odom_.twist.twist.angular.z = w;
    odom_.twist.twist.linear.x = dx/seconds_since_last_update;
    odom_.twist.twist.linear.y = dy/seconds_since_last_update;
}

void AckermanSteer::publishOdometry ( double step_time )
{

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
    std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

    tf::Quaternion qt;
    tf::Vector3 vt;

    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );

    }
    if ( odom_source_ == WORLD ) {
        // getting data from gazebo world
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = model->WorldPose();
#else
        ignition::math::Pose3d pose = model->GetWorldPose().Ign();
#endif
        qt = tf::Quaternion ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );
        vt = tf::Vector3 ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
        ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
        linear = model->WorldLinearVel();
        odom_.twist.twist.angular.z = model->WorldAngularVel().Z();
#else
        linear = model->GetWorldLinearVel().Ign();
        odom_.twist.twist.angular.z = model->GetWorldAngularVel().Ign().Z();
#endif

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.Rot().Yaw();
        odom_.twist.twist.linear.x = cosf ( yaw ) * linear.X() + sinf ( yaw ) * linear.Y();
        odom_.twist.twist.linear.y = cosf ( yaw ) * linear.Y() - sinf ( yaw ) * linear.X();
    }

    if (publishOdomTF_ == true){
        tf::Transform base_footprint_to_odom ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( base_footprint_to_odom, current_time,
                                   odom_frame, base_footprint_frame ) );
    }


    // set covariance
    odom_.pose.covariance[0] = 0.0;
    odom_.pose.covariance[7] = 0.0;
    odom_.pose.covariance[14] = 0.0;
    odom_.pose.covariance[21] = 0.0;
    odom_.pose.covariance[28] = 0.0;
    odom_.pose.covariance[35] = 0.0;


    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish ( odom_ );
}

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AckermanSteer)

} // namespace gazebo
