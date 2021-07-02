clear all;
clc;
close all;

%% preamble

% inform where the phd-codes folder is located for finding related code
folder_phd_codes = '/home/filipe/gitSources/doc/phd-codes';

addpath(strcat(folder_phd_codes,'/implementing/lib/dq'));
addpath(strcat(folder_phd_codes,'/implementing/modelling'));
addpath('./lib');


%% ROS

% initializing ros connection
ros_gentle_init();

% loading ros rosi topics
r_tpc = rosi_ros_topic_info();

% subscribing to ros topics
sub_arms_pos = rossubscriber(r_tpc.arms_pos.adr, r_tpc.arms_pos.msg);
sub_arms_torque = rossubscriber(r_tpc.arms_torque.adr, r_tpc.arms_torque.msg);


%% Usefulments

% loading robot dual quaternion model
load(strcat(folder_phd_codes,'/implementing/modelling/', 'model_dq.mat'));


%% Infinit loop

while true
   
    % receiving ros data
    data_arms_pos = receive(sub_arms_pos, 3);
    data_arms_torque = receive(sub_arms_torque, 3 );
    
    arms_pos = rosi_correct_joints_signal(data_arms_pos.JointVar)
    arms_torque = rosi_correct_joints_signal(data_arms_torque.JointVar)
    
end























