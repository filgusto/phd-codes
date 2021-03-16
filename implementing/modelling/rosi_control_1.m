% Code based on the snippet contained at the article
% Adorno, 2020, DQ Robotics: a Library for Robot Modeling and Control
clear all;
clc;

%% Preamble

% adding subfolder to path
addpath('./lib/');

% loading modelling variables
load('model_dq_1.mat');

%% ROS 

% initializing ros connection
ros_gentle_init();

% subscribing to topics
sub_manipulator_joints_pos = rossubscriber('/manipulator/sensor/joints_pos');
sub_rosi_traction_pos = rossubscriber('/rosi/sensor/traction_position');

%% Control

% initializing the error vector
e = ones(8,1);

% control loop
while norm(e) > 0.001
    
    % receiving robot joints vector from ROS
    q = ros_retrieve_joints(sub_rosi_traction_pos, sub_manipulator_joints_pos);
    
    % J = robot.pose_jacobian(q);

    % x = robot.fkm(q);

    % e = vec8(x-xd);

    % u = -pinv(J)*gain*e;

    % TODO send u to the simulator
    
end







% shutting down ros connection
rosshutdown;









