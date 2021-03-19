% Code based on the snippet contained at the article
% Adorno, 2020, DQ Robotics: a Library for Robot Modeling and Control
clear all;
clc;
close all;

%% preamble

% inform where the phd-codes folder is located for finding related code
folder_phd_codes = '/home/filipe/gitSources/doc/phd-codes';

% adding subfolder to path
addpath('./lib/');
addpath(strcat(folder_phd_codes,'/prototyping/dq-robotics/lib')); 
addpath(strcat(folder_phd_codes,'/matlab/lib/plot'));

% loading modelling variables
load('model_dq_1.mat');

%% parameters

% declaring set-point
r = 0.0191 + 0*i_ + 0.9999*j_ + 0.0*k_;
p = -0.5398* i_ + 0* j_ + 1.4678* k_;
xd = r + E_ *0.5* p*r;

% controller gain
gain = 1;

% error tolerance
error_tol = 0.01;

%% ROS 

% initializing ros connection
ros_gentle_init();

% subscribing to topics
sub_manipulator_joints_pos = rossubscriber('/manipulator/sensor/joints_pos', 'sim_rosi/ManipulatorJoints');
sub_rosi_pose = rossubscriber('/rosi/cheat/rosi_pose', 'geometry_msgs/PoseStamped');

pub_traction_speed = rospublisher('/rosi/cmd/traction_speed', 'sim_rosi/RosiMovementArray');
pub_manipulator_speed = rospublisher('/manipulator/cmd/joints_vel_target', 'sim_rosi/ManipulatorJoints'); % TODO manipulador deve aceitar comando de velocidade


%% Control

% initializing the error vector
e = ones(8,1);

% control loop
hist_q = {};
hist_x = {};
hist_xd = {};
hist_e = {};
hist_u = {};


i_f = 1;
while norm(e) > error_tol
    
    %% updating variables
    
    % receiving robot joints vector from ROS
    q = ros_retrieve_joints(sub_rosi_pose, sub_manipulator_joints_pos);
    
    %% control update
    
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
    
    %% sending command to the robot
    
    % sends the commands to the robot
    ros_send_robot_cmd_vel(u, pub_traction_speed, pub_manipulator_speed);
    
    %% saving varibles history
    hist_q{i_f} = q;
    hist_x{i_f} = x;
    hist_xd{i_f} = xd;
    hist_e{i_f} = e;
    hist_u{i_f} = u;
    
    %% integrating iterative variable
    i_f = i_f + 1;
end


% shutting down ros connection
rosshutdown;









