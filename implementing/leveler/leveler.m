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

% subscribing
sub_imu = rossubscriber(r_tpc.imu.adr, r_tpc.imu.msg);


%% Usefulments

% set-point
f_world = DualQuaternion([1,0,0,0,0,0,0,0]);

% loading robot dual quaternion model
load(strcat(folder_phd_codes,'/implementing/modelling/', 'model_dq.mat'));


%% Loop

disp('Loop initiated...');
while true
   
    % receiving imu ros msg
    data = receive(sub_imu, 3);
    
    % treat imu data
    [dq_ori_world_base, omega_world_base_world, accel_world_base_base] ...
        = treat_imu_data(data);
    
    % computing lever joints w.r.t. the base 
    dq_world_laj1 = dq_ori_world_base * dq_base_laj1;
    dq_world_laj2 = dq_ori_world_base * dq_base_laj2;
    dq_world_laj3 = dq_ori_world_base * dq_base_laj3;
    dq_world_laj4 = dq_ori_world_base * dq_base_laj4;
    
    % plotting 
    plot([f_world, dq_ori_world_base, ...
        dq_world_laj1, dq_world_laj2, dq_world_laj3, dq_world_laj4]);
    
end














