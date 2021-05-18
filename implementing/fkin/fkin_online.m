clear all;
clc;
close all;

%% preamble

% inform where the phd-codes folder is located for finding related code
folder_phd_codes = '/home/filipe/gitSources/doc/phd-codes';

addpath(strcat(folder_phd_codes,'/implementing/modelling'));
addpath(strcat(folder_phd_codes,'/implementing/lib/dq'));
addpath('./lib');

% loading robot dualquaternion modules
load('model_dq.mat');

%% parameters

% test time length
test_length = 20;       % seconds

% manipulator dof
arm_dof = 7;


%% ROS 

% initializing ros connection
ros_gentle_init();

% loading ros rosi topics
r_tpc = rosi_ros_topic_info();

% subscribing to topics
sub_pos_manipulator_joints = rossubscriber(r_tpc.pos_joints.adr, r_tpc.pos_joints.msg);
sub_pose_rosi = rossubscriber(r_tpc.pose_rosi.adr, r_tpc.pose_rosi.msg);
sub_pose_tcp = rossubscriber(r_tpc.pose_tcp.adr, r_tpc.pose_tcp.msg);

%% Fkin Loop

% vector
time_arr = [];
e_theta = [];
e_pos = [];

% initializations
tic;        % initializing temporizations
flag_end = false;       % flag that inidicates flag of operation
l_i = 1;

disp('Loop initiated...');
while ~ flag_end
    
    % current time 
    time_arr(l_i) = toc;
    
    %% Retrieving ROS data
    
    % robot joints vector from ROS
    q = ros_retrieve_mani_joints(sub_pos_manipulator_joints);
    
    % rosi base pose
    dq_world_base = ros_retrieve_dq(sub_pose_rosi);
    
    % tcp pose
    dq_world_tcp = ros_retrieve_dq(sub_pose_tcp);
     
    
    %% Performing fkin
    dq_res = dq_rosi_fkin(q, dq_world_base,dq_base_arm, dq_arm_arr, dq_j6_tcp);
    
    %% Comparison with actual EE pose

    % computing the error vector
    e = dq_world_tcp.conj * dq_res;

    % obtaining error metrics
    e_theta(l_i) = q_extract_angle_n_director(e.q_p);
    e_pos(l_i)   = q_extract_pos_error_norm(e.q_d);
    
    % only display current result
    e_theta(l_i);
    e_pos(l_i);
    
    %% Test finish condition
   
    if time_arr(l_i) > test_length
       flag_end = true; 
    end
    
    % updating iterator
    l_i = l_i+1;
    
    plot([DualQuaternion dq_res dq_world_tcp])

end


%% Plotting
figure
hold on;

subplot(2,1,1);
plot(time_arr, e_theta, '*b');
title('Orientation error');
xlabel('time [s]');
ylabel('error mag [rad]');
grid on;

subplot(2,1,2);
plot(time_arr, e_pos, '*r');
title('Position error');
xlabel('time [s]');
ylabel('error mag [m]');
grid on;








