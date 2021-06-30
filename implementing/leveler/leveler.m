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
dq_f_world = DualQuaternion([1,0,0,0,0,0,0,0]);

% loading robot dual quaternion model
load(strcat(folder_phd_codes,'/implementing/modelling/', 'model_dq.mat'));


%% Loop

% defining robot frame director vector
dq_base_director = DualQuaternion.pureTranslation([1 0 0]);
dq_base_rotating_axis = DualQuaternion.pureTranslation([0 0 1]);

disp('Loop initiated...');
while true
   
    % receiving imu ros msg
    data = receive(sub_imu, 3);
    
    % treat imu data
    [dq_ori_world_base, omega_world_base_world, accel_world_base_base] ...
        = treat_imu_data(data);
    
    % find x-aligned dq_pose
    dq_x_align = get_dq_frame_x_aligned(dq_ori_world_base,...
                                        dq_base_director, ...
                                        dq_base_rotating_axis);
    
    % computing pose error w.r.t. x-aligned frame
    dq_error = dq_x_align.conj *  dq_ori_world_base;
    
    
    % extracting transform components
    [dq_error_tr, dq_error_n, dq_error_theta] = dq_error.extractTransformComponents;        
   
    % computing lever joints w.r.t. the error frame
    dq_world_laj1 = dq_error * dq_base_laj1;
    dq_world_laj2 = dq_error * dq_base_laj2;
    dq_world_laj3 = dq_error * dq_base_laj3;
    dq_world_laj4 = dq_error * dq_base_laj4;
    
    
    %% plotting
    
    dq_error
    error_n = dq_error_n
    error_theta = dq_error_theta
    error_theta_g = rad2deg(error_theta)
    
%      plot([dq_f_world, dq_ori_world_base]);
%     hold on;  
%     plot(dq_x_align);   
      plot(dq_error)
      hold on;
%       plot([dq_world_laj1, dq_world_laj2, dq_world_laj3, dq_world_laj4]);
      quiver3(0, 0, 0, error_n(1), error_n(2), error_n(3),'-m');
      hold off;

    
end














