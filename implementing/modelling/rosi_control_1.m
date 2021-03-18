% Code based on the snippet contained at the article
% Adorno, 2020, DQ Robotics: a Library for Robot Modeling and Control
clear all;
clc;

%% preamble

% adding subfolder to path
addpath('./lib/');

% loading modelling variables
load('model_dq_1.mat');

%% parameters

% declaring set-point
r = 0.0 - 0.5509*i_ - 0.0342*j_ + 1.0364*k_;
p = -0.5509* i_ + -1.0342* j_ + 1.0364* k_;
xd = r + E_ *0.5* p*r;

% controller gain
gain = 10;

% error tolerance
error_tol = 0.01;

%% ROS 

% initializing ros connection
ros_gentle_init();

% subscribing to topics
sub_manipulator_joints_pos = rossubscriber('/manipulator/sensor/joints_pos', 'sim_rosi/ManipulatorJoints');
sub_rosi_pose = rossubscriber('/rosi/cheat/rosi_pose', 'geometry_msgs/PoseStamped');

pub_traction_speed = rospublisher('/rosi/command_traction_speed', 'sim_rosi/RosiMovementArray');
pub_ % TODO manipulador deve aceitar comando de velocidade


%% Control

% initializing the error vector
e = ones(8,1);

% control loop
while norm(e) > error_tol
    
    % receiving robot joints vector from ROS
    q = ros_retrieve_joints(sub_rosi_pose, sub_manipulator_joints_pos);
    
    % obtains the jacobian relating joints position and
    % end-effector dual quaternion terms velocities
    J = robot.pose_jacobian(q);
    
    % obtains the mobile base jacobian, relating wheels velocities to the
    % configuration velocities
    J_base = constraint_jacobian_custom(q, wheel_radius, wheels_side_distance);

    % obtains the end-effector pose state given the joints state
    x = robot.fkm(q);

    % directly computes error by subtracting the current pose dual
    % quaternion by the desired pose dual quaternion
    e = vec8(x-xd);

    % mapping wheels speed to configuration space velocities
    u = -pinv(J)*gain*e;

    % TODO send u to the simulator
    
    disp('hi');
    
end







% shutting down ros connection
rosshutdown;









