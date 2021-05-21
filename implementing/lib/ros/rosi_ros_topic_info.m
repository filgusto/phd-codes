function r_tpc = rosi_ros_topic_info()

% Defines ROSI simulator topics in variables for brevity in recalling

%% ROSI

% ROSI - Sub - Pose - CHEAT
r_tpc.pose_rosi.adr = '/rosi/cheat/pose_rosi';
r_tpc.pose_rosi.msg = 'bj_libraries/DualQuaternionStamped';

% ROSI - Sub - Pose Vel - Cheat
r_tpc.pose_vel_rosi.adr = '/rosi/cheat/pose_vel_rosi';
r_tpc.pose_vel_rosi.msg = 'bj_libraries/DualQuaternionStamped';

% ROSI - Sub - Mani base pose vel - Cheat
r_tpc.pose_vel_mani.adr = '/rosi/cheat/pose_vel_mani';
r_tpc.pose_vel_mani.msg = 'bj_libraries/DualQuaternionStamped';

% ROSI arms - Sub - Angular position
r_tpc.pos_arms.adr = '/rosi/sensor/pos_arms';
r_tpc.pos_arms.msg = 'sim_rosi/RosiMovementArray';

% ROSI traction - Sub - Angular position
r_tpc.pos_traction.adr = '/rosi/sensor/pos_traction';
r_tpc.pos_traction.msg = 'sim_rosi/RosiMovementArray';

% ROSI GPS - Sub - NavSat info
r_tpc.gps.adr = '/sensor/gps';
r_tpc.gps.msg = 'sensor_msgs/NavSatFix';

% ROSI IMU - Sub - Gyro info
r_tpc.imu.adr = '/sensor/imu';
r_tpc.imu.msg = 'sensor_msgs/Imu';

% ROSI arms - Pub - Angular speed
r_tpc.speed_arms.adr = '/rosi/cmd/speed_arms';
r_tpc.speed_arms.msg = 'sim_rosi/RosiMovementArray';


% ROSI traction - Pub - Angular speed
r_tpc.speed_traction.adr = '/rosi/cmd/speed_traction';
r_tpc.speed_traction.msg = 'sim_rosi/RosiMovementArray';



%% Manipulator

% Manipulator joints - Pub - Position target
r_tpc.pos_target_joints.adr = '/manipulator/cmd/pos_target_joints';
r_tpc.pos_target_joints.msg = 'sim_rosi/ManipulatorJoints';

% Manipulator joints - Pub - Velocity target
r_tpc.vel_target_joints.adr = '/manipulator/cmd/vel_target_joints';
r_tpc.vel_target_joints.msg = 'sim_rosi/ManipulatorJoints';


% Manipulator TCP pose
r_tpc.pose_vel_tcp.adr = '/manipulator/cheat/pose_vel_tcp';
r_tpc.pose_vel_tcp.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator Joints - Sub - Angular position - 
r_tpc.pos_joints.adr = '/manipulator/sensor/pos_joints';
r_tpc.pos_joints.msg = 'sim_rosi/ManipulatorJoints';

% Manipulator Joints - Sub - Angular velocity - 
r_tpc.vel_joints.adr = '/manipulator/sensor/vel_joints';
r_tpc.vel_joints.msg = 'sim_rosi/ManipulatorJoints';

% Manipulator Wrist - Sub - Force and torque
r_tpc.ft_wrist.adr = '/manipulator/sensor/ft_wrist';
r_tpc.ft_wrist.msg = 'geometry_msgs/TwistStamped';

% Manipulator joints - Sub - Torque
r_tpc.torque_joints.adr = '/manipulator/sensor/torque_joints';
r_tpc.torque_joints.msg = 'sim_rosi/ManipulatorJoints';

% Manipulator tcp - Sub - Pose - CHEAT
r_tpc.pose_tcp.adr = '/manipulator/cheat/pose_tcp';
r_tpc.pose_tcp.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator base - Sub - Pose w.r.t. rosi base - CHEAT
r_tpc.pose_mani_base.adr = '/rosi/cheat/pose_mani_base';
r_tpc.pose_mani_base.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 0 - Sub - Pose - Cheat
r_tpc.pose_mani_joint0.adr = '/rosi/cheat/pose_mani_joint1';
r_tpc.pose_mani_joint0.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 1 - Sub - Pose - Cheat
r_tpc.pose_mani_joint1.adr = '/rosi/cheat/pose_mani_joint2';
r_tpc.pose_mani_joint1.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 2 - Sub - Pose - Cheat
r_tpc.pose_mani_joint2.adr = '/rosi/cheat/pose_mani_joint3';
r_tpc.pose_mani_joint2.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 3 - Sub - Pose - Cheat
r_tpc.pose_mani_joint3.adr = '/rosi/cheat/pose_mani_joint4';
r_tpc.pose_mani_joint3.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 4 - Sub - Pose - Cheat
r_tpc.pose_mani_joint4.adr = '/rosi/cheat/pose_mani_joint5';
r_tpc.pose_mani_joint4.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 5 - Sub - Pose - Cheat
r_tpc.pose_mani_joint5.adr = '/rosi/cheat/pose_mani_joint6';
r_tpc.pose_mani_joint5.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 6 - Sub - Pose - Cheat
r_tpc.pose_mani_joint6.adr = '/rosi/cheat/pose_mani_joint7';
r_tpc.pose_mani_joint6.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 0 - Sub - Pose Vel - Cheat
r_tpc.pose_vel_mani_joint0.adr = '/manipulator/cheat/pose_mani_joint_vel0';
r_tpc.pose_vel_mani_joint0.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 1 - Sub - Pose Vel - Cheat
r_tpc.pose_vel_mani_joint1.adr = '/manipulator/cheat/pose_mani_joint_vel1';
r_tpc.pose_vel_mani_joint1.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 2 - Sub - Pose Vel - Cheat
r_tpc.pose_vel_mani_joint2.adr = '/manipulator/cheat/pose_mani_joint_vel2';
r_tpc.pose_vel_mani_joint2.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 3 - Sub - Pose Vel - Cheat
r_tpc.pose_vel_mani_joint3.adr = '/manipulator/cheat/pose_mani_joint_vel3';
r_tpc.pose_vel_mani_joint3.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 4 - Sub - Pose Vel - Cheat
r_tpc.pose_vel_mani_joint4.adr = '/manipulator/cheat/pose_mani_joint_vel4';
r_tpc.pose_vel_mani_joint4.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 5 - Sub - Pose Vel - Cheat
r_tpc.pose_vel_mani_joint5.adr = '/manipulator/cheat/pose_mani_joint_vel5';
r_tpc.pose_vel_mani_joint5.msg = 'bj_libraries/DualQuaternionStamped';

% Manipulator joint 6 - Sub - Pose Vel - Cheat
r_tpc.pose_vel_mani_joint6.adr = '/manipulator/cheat/pose_mani_joint_vel6';
r_tpc.pose_vel_mani_joint6.msg = 'bj_libraries/DualQuaternionStamped';



%% Scene

% Setpoint Dummy SP 1 - Sub - Pose - CHEAT
r_tpc.pose_sp.adr = '/sim/pose/f_sp1';
r_tpc.pose_sp.msg = 'bj_libraries/DualQuaternionStamped';

% Simulation Time - Sub - Time
r_tpc.sim_time.adr = '/sim/time';
r_tpc.sim_time.msg = 'std_msgs/Header';


end

