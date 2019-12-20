# Gazebo_AckermanVehicle

这个pkg的主要内容是实现了阿克曼转向车辆在Gazebo、Ros和Rviz的仿真。

## 功能介绍

**你可以直接启动在根目录下的gazebo.launch文件进行gazebo的仿真**

**车辆的运动默认接受来自于”cmd_vel“话题的msg，msg的格式geometry_msgs::Twist**

```
 Vx= cmd_msg->linear.x

 Vrot= cmd_msg->angular.z
```

上述中，**linear.x**表示的是车辆的期望线速度;**cmd_msg->angular.z**表示的是车辆的期望转角。

**Tips:**在gazebo中仿真运行时需要下载gazebo的模型文件，否则打不开world。

[模型文件的下载](https://blog.csdn.net/qq_36170626/article/details/90417359)

## 主要功能包的介绍

### 1.ackermansteer包

这个功能包主要存放了自己修改过的ackerman转向的gazebo插件以及gazebo的仿真模型，分别存放于

ackermansteer文件夹和steerbot文件夹。

你可以通过修改steerbot文件夹中的urdf模型进行各项参数的修改。

```
 <gazebo>
      <plugin name="ackermansteer" filename="libackermansteer.so">
         <debug>false</debug>
         <commandTopic>cmd_vel</commandTopic>
         <odometryTopic>odom</odometryTopic>
         <odometryFrame>odom</odometryFrame>
         <robotBaseFrame>base_link</robotBaseFrame>
         <publishWheelTF>false</publishWheelTF>
         <publishSteerTF>false</publishSteerTF>
         <publishOdomTF>false</publishOdomTF>
	     <publishWheelJointState>false</publishWheelJointState>
	     <legacyMode>true</legacyMode>
	     <odometryFrame>false</odometryFrame>
         <rosDebugLevel></rosDebugLevel>
         <odometrySource>true</odometrySource>
         <publishTf>1</publishTf>
         
      <!-- Wheel Placement -->
         <wheelSeparation>1.535</wheelSeparation>
         <wheelbase>3.271</wheelbase>
         <wheelDiameter>0.8693</wheelDiameter>

         <wheelAcceleration>0.0</wheelAcceleration>
         <wheelTorque>5000000.0</wheelTorque>
         <updateRate>50.0</updateRate>
 	     <drive_init_velocity>0.0</drive_init_velocity>
 	  <!--PID settings - - - - - - - - - - - - - - - - - - - -
              p      -    proportional gain
              i      -    intergral gain
              d      -    derivative gain
              imax   -    anti windup max
              imin   -    anti windup min
         - - - - - - - - - - - - - - - - - - - - - - - - - -  -->

         <!-- Steering PID settings -->
         <steer_p>50000.0</steer_p>
         <steer_i>5.0</steer_i>
         <steer_d>0.11</steer_d>
         <steer_imax>5.0</steer_imax>
         <steer_imin>-5.0</steer_imin>
         <steer_max_effort>100000.0</steer_max_effort>
         <steer_init_angle>0.0</steer_init_angle>

         <!-- Wheel Velocity PID settings -->
         <drive_p>50000.0</drive_p>
         <drive_i>0.01</drive_i>
         <drive_d>0.01</drive_d>
         <drive_imax>1.0</drive_imax>
         <drive_imin>-1.0</drive_imin>
         <drive_max_effort>100000.0</drive_max_effort>
         <drive_init_velocity>0.0</drive_init_velocity>
         
         <!-- Wheel Joints -->
         <FR_steerJoint>front_right_steer_bearing</FR_steerJoint>
         <FR_driveJoint>front_right_wheel_bearing</FR_driveJoint>
         <FL_steerJoint>front_left_steer_bearing</FL_steerJoint>
         <FL_driveJoint>front_left_wheel_bearing</FL_driveJoint>
         <RL_steerJoint>rear_left_steer_bearing</RL_steerJoint>
         <RL_driveJoint>rear_left_wheel_bearing</RL_driveJoint>
         <RR_steerJoint>rear_right_steer_bearing</RR_steerJoint>
         <RR_driveJoint>rear_right_wheel_bearing</RR_driveJoint>

      </plugin>
    </gazebo>
 	 	 
```

### 2.**odom_publisher包**

该包主要实现了发送车辆坐标相对于odom坐标的位置，以在Rviz中显示车辆模型在gazebo的实际位置。

### 3.topic_demo包

该包主要实现了车辆速度和角度信息的发送，用于车辆的调试，可以直接启动该节点进行调试。

