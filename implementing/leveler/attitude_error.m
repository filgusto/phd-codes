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
sub_arms_pos = rossubscriber(r_tpc.arms_pos.adr, r_tpc.arms_pos.msg);

% publishing
pub_arms_pos = rospublisher(r_tpc.position_arms.adr, r_tpc.position_arms.msg);

%% Usefulments

% set-point
dq_f_world = DualQuaternion([1,0,0,0,0,0,0,0]);

% loading robot dual quaternion model
load(strcat(folder_phd_codes,'/implementing/modelling/', 'model_dq.mat'));

%% Sending lever arms to home position

% receiving ROS time
ros_time = rostime('now');

 % arms position home position 
cmd_arms_home = [pi/2, pi/2, pi/2, pi/2].';

 % corrects positions for applying at ROSI with defined signals
cmd_arms_home = rosi_correct_joints_signal(cmd_arms_home);

% mounting and publishing ROS msg
cmd_arms_home_msg = rosi_mount_arms_pos_msg(cmd_arms_home, ros_time);
send(pub_arms_pos, cmd_arms_home_msg);

pause(2);

%% Loop

% defining robot frame director vector
dq_base_director = DualQuaternion.pureTranslation([1 0 0]);
dq_base_rotating_axis = DualQuaternion.pureTranslation([0 0 1]);

disp('Loop initiated...');
while true
   
    %% ROS
    
    % receiving ROS time
    ros_time = rostime('now');
    
    % receiving ros data
    data_imu = receive(sub_imu, 3);
    data_arms_pos = receive(sub_arms_pos, 3);
    
    % treat imu data
    [dq_ori_world_base, omega_world_base_world, accel_world_base_base] ...
        = treat_imu_data(data_imu);
    
    %% Computing attitude error vector for each arm joint
    
    % computing attitude error metrics
     dq_error = get_attitude_error(dq_ori_world_base, dq_base_director, dq_base_rotating_axis);    
                                         
    % extracting transform components
    [dq_error_tr, dq_error_n, dq_error_theta] = dq_error.extractTransformComponents;    
    
    %% Computing the error vector projection on each arm x vector
    
    % computing lever joints w.r.t. the error frame
    dq_r_laj1 = dq_error * dq_base_laj1;
    dq_r_laj2 = dq_error * dq_base_laj2;
    dq_r_laj3 = dq_error * dq_base_laj3;
    dq_r_laj4 = dq_error * dq_base_laj4;
    
    % computing error vector for each arms axis
    v_err_r_laj1 = cross(dq_r_laj1.extractTranslation, dq_error_n);
    v_err_r_laj2 = cross(dq_r_laj2.extractTranslation, dq_error_n);
    v_err_r_laj3 = cross(dq_r_laj3.extractTranslation, dq_error_n);
    v_err_r_laj4 = cross(dq_r_laj4.extractTranslation, dq_error_n);
    
    % normalizing v_err
    v_err_r_laj1 = v_err_r_laj1 / norm(v_err_r_laj1);
    v_err_r_laj2 = v_err_r_laj2 / norm(v_err_r_laj2);
    v_err_r_laj3 = v_err_r_laj3 / norm(v_err_r_laj3);
    v_err_r_laj4 = v_err_r_laj4 / norm(v_err_r_laj4);
  
    % normalizing theta error by pi
    error_theta_normalized = abs(dq_error_theta / pi);
     
    % modulating accordingly to the theta error
    v_err_r_laj1 = v_err_r_laj1 * error_theta_normalized;
    v_err_r_laj2 = v_err_r_laj2 * error_theta_normalized;
    v_err_r_laj3 = v_err_r_laj3 * error_theta_normalized;
    v_err_r_laj4 = v_err_r_laj4 * error_theta_normalized;
    
    %% Obtaining the error vector projection on each arm unitary x vector
    
    % obtaining rotated vertical axis of each arm
    % as the the x vector is vertical in all arms, and each arm joint is
    % rigidly fixed to the robot base, one considers the same vertical
    % vector for all lever joints
    v_x_levers = get_lever_vertical([1 0 0], dq_r_laj1);  % x unit vector is the vertical for all levers
    
    % projecting the error vector for all lever arm joints
    [v_err_r_laj1_x_proj, v_err_r_laj1_sig] = vector_projection(v_err_r_laj1, v_x_levers);
    [v_err_r_laj2_x_proj, v_err_r_laj2_sig] = vector_projection(v_err_r_laj2, v_x_levers);
    [v_err_r_laj3_x_proj, v_err_r_laj3_sig] = vector_projection(v_err_r_laj3, v_x_levers);
    [v_err_r_laj4_x_proj, v_err_r_laj4_sig] = vector_projection(v_err_r_laj4, v_x_levers);
    
    
    %% Control
        
    % Extracting arms current position
    arms_pos_current = rosi_correct_joints_signal(data_arms_pos.JointVar);
    
    % control k
    ctrl_k = 5;
    
    % computing differential position command
    cmd_laj = [ctrl_k * (v_err_r_laj1_sig * norm(v_err_r_laj1_x_proj));...
               ctrl_k * (v_err_r_laj2_sig * norm(v_err_r_laj2_x_proj));...
               ctrl_k * (v_err_r_laj3_sig * norm(v_err_r_laj3_x_proj));...
               ctrl_k * (v_err_r_laj4_sig * norm(v_err_r_laj4_x_proj))];
    
    % summing to current home position
    cmd_arms_pos = arms_pos_current + cmd_laj;   
                
    % clipps the command if outside predetermined bounds
    cmd_arms_pos_clp = rosi_clip_lever_pos(cmd_arms_pos);
    
    % corrects positions for applying at ROSI with defined signals
    cmd_arms_pos_corr = rosi_correct_joints_signal(cmd_arms_pos_clp.');
        
    % Only sends the control signal if there is a significant error
    if sum(isnan(cmd_arms_pos_corr)) == 0
    
        % mounting and publishing ROS msg
        cmd_arms_pos_msg = rosi_mount_arms_pos_msg(cmd_arms_pos_corr, ros_time);
        send(pub_arms_pos, cmd_arms_pos_msg);
        
        % DELETE-ME
        cmd_arms_pos_g = rad2deg(cmd_arms_pos)
    end

    
    %% plotting
    
    dq_error;
    error_n = dq_error_n;
    error_theta = dq_error_theta;
    error_theta_g = rad2deg(error_theta);
    
    
    % modulating error only for plotting purposes
    mod = 5;
    
    v_err_r_laj1 = v_err_r_laj1 * mod;
    v_err_r_laj2 = v_err_r_laj2 * mod;
    v_err_r_laj3 = v_err_r_laj3 * mod;
    v_err_r_laj4 = v_err_r_laj4 * mod;
    
    v_err_r_laj1_x_proj = v_err_r_laj1_x_proj * mod;
    v_err_r_laj2_x_proj = v_err_r_laj2_x_proj * mod;
    v_err_r_laj3_x_proj = v_err_r_laj3_x_proj * mod;
    v_err_r_laj4_x_proj = v_err_r_laj4_x_proj * mod;
    
    % extracting each lever arm vector
    v_laj1 = dq_r_laj1.extractTranslation;
    v_laj2 = dq_r_laj2.extractTranslation;
    v_laj3 = dq_r_laj3.extractTranslation;
    v_laj4 = dq_r_laj4.extractTranslation;
    
    
%      plot([dq_f_world, dq_ori_world_base]);
%     hold on;  
%     plot(dq_x_align);   
      plot(dq_error)
      hold on;
      plot([dq_r_laj1, dq_r_laj2, dq_r_laj3, dq_r_laj4]);
      quiver3(0, 0, 0, error_n(1), error_n(2), error_n(3),'-m');
      
      %% Debug plot
      
%       quiver3(v_laj1(1), v_laj1(2), v_laj1(3), v_err_r_laj1(1), v_err_r_laj1(2), v_err_r_laj1(3),'-k');
%       quiver3(v_laj2(1), v_laj2(2), v_laj2(3), v_err_r_laj2(1), v_err_r_laj2(2), v_err_r_laj2(3),'-k');
%       quiver3(v_laj3(1), v_laj3(2), v_laj3(3), v_err_r_laj3(1), v_err_r_laj3(2), v_err_r_laj3(3),'-k');
%       quiver3(v_laj4(1), v_laj4(2), v_laj4(3), v_err_r_laj4(1), v_err_r_laj4(2), v_err_r_laj4(3),'-k');
      
      quiver3(v_laj1(1), v_laj1(2), v_laj1(3), v_err_r_laj1_x_proj(1), v_err_r_laj1_x_proj(2), v_err_r_laj1_x_proj(3),'-k');
      quiver3(v_laj2(1), v_laj2(2), v_laj2(3), v_err_r_laj2_x_proj(1), v_err_r_laj2_x_proj(2), v_err_r_laj2_x_proj(3),'-k');
      quiver3(v_laj3(1), v_laj3(2), v_laj3(3), v_err_r_laj3_x_proj(1), v_err_r_laj3_x_proj(2), v_err_r_laj3_x_proj(3),'-k');
      quiver3(v_laj4(1), v_laj4(2), v_laj4(3), v_err_r_laj4_x_proj(1), v_err_r_laj4_x_proj(2), v_err_r_laj4_x_proj(3),'-k');
      
      
      hold off;

    
end














