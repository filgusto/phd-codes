% Code based on the snippet contained at the article
% Adorno, 2020, DQ Robotics: a Library for Robot Modeling and Control
clear all;
clc;
close all;

preamble;

%% world frame
wf = (1 + 0*i_ + 0*j_ + 0*k_) + (0 + 0*i_ + 0*j_ + 0*k_)*E_;

%% ROS

% subscribing to pose topics
sub_robot_pose = rossubscriber('/rosi/cheat/rosi_pose', 'bj_libraries/DualQuaternionStamped');
sub_mani_base_pose = rossubscriber('/rosi/cheat/mani_base_pose', 'bj_libraries/DualQuaternionStamped');
sub_mani_tcp_pose = rossubscriber('/rosi/cheat/tcp_pose', 'bj_libraries/DualQuaternionStamped');
sub_sp = rossubscriber('/sim/pose/dummy_1','bj_libraries/DualQuaternionStamped');

% receiving messages
robot_pose_data = receive(sub_robot_pose, 3);
mani_base_pose_data = receive(sub_mani_base_pose, 3);
mani_tcp_pose_data = receive(sub_mani_tcp_pose, 3);
sp_pose_data = receive(sub_sp, 3);

% converting to dual quaternion library language
robot_pose = dq2dqmat(robot_pose_data);
mani_base_pose = dq2dqmat(mani_base_pose_data);
mani_tcp_pose = dq2dqmat(mani_tcp_pose_data);
sp_pose = dq2dqmat(sp_pose_data);

% plotting
figure
hold on;
axis equal;
view(3);
xlabel('X'); 
ylabel('Y');
zlabel('Z');
grid on;
plot(wf);
plot(robot_pose.normalize);
plot(mani_base_pose.normalize);
plot(mani_tcp_pose.normalize);
plot(sp_pose.normalize);





