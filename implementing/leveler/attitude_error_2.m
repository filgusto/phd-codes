clear all;
clc;
close all;

%% parameters

% set-point
dq_f_world = DualQuaternion([1,0,0,0,0,0,0,0]);

% orientation setpoint in rpy format
dq_ori_sp = DualQuaternion.pureRotation(deg2rad([0 0 0]));

% arms home angular position
cmd_arms_home = [pi/2, pi/2, pi/2, pi/2].';
% cmd_arms_home = [0, 0, 0, 0].';

% control k
ctrl_k = 1;

% defining clipping angles
la_angle_min = deg2rad(15);
la_angle_max = deg2rad(165);

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

% loading robot dual quaternion model
load(strcat(folder_phd_codes,'/implementing/modelling/', 'model_dq.mat'));

%% Sending lever arms to home position

% receiving ROS time
ros_time = rostime('now');

 % corrects positions for applying at ROSI with defined signals
cmd_arms_home = rosi_correct_joints_signal(cmd_arms_home);

% mounting and publishing ROS msg
cmd_arms_home_msg = rosi_mount_arms_pos_msg(cmd_arms_home, ros_time);
send(pub_arms_pos, cmd_arms_home_msg);

% small pause for waiting start position command 
disp('Sending home command and pausing...');
pause(1);
disp('Resuming!')

%% Logging variables

log_time = [];
log_laj_pos = [];
log_error_theta = [];
log_laj_delta_sig = [];
log_laj_cmd_pos = [];

%% Setting figure location and size
f1 = figure();
f1.Position = [570 0 750 1000];

%% Loop

% defining robot frame director vector
dq_base_director = DualQuaternion.pureTranslation([1 0 0]);
dq_base_rotating_axis = DualQuaternion.pureTranslation([0 0 1]);

disp('Loop STARTED!');

while true

    %% ROS

    % receiving ROS time
    ros_time = rostime('now');

    % receiving ros data
    data_imu = receive(sub_imu, 3);
    data_arms_pos = receive(sub_arms_pos, 3);

    % treat imu data
    [dq_world_base, omega_world_base_world, accel_world_base_base] ...
        = treat_imu_data(data_imu);

    %% Computing attitude error vector for each arm joint

    % cancelling yaw component by rotating the input pose dual quaternion
    dq_world_base_noyaw = remove_yaw(dq_world_base);

    % computing the attitude pose error
    dq_error = dq_ori_sp.conj * dq_world_base_noyaw;

    % extracting transform components
    [dq_error_n, dq_error_theta] = dq_error.extractRotationDirectorVecAndAngle();


    %% Computing the error vector on each arm x vector

    % error orientation
    q_error = dq_error.q_p;

    % computing lever joints w.r.t. the error frame using dual quaternion
    dq_r_laj1 = dq_error * dq_base_laj1;
    dq_r_laj2 = dq_error * dq_base_laj2;
    dq_r_laj3 = dq_error * dq_base_laj3;
    dq_r_laj4 = dq_error * dq_base_laj4;

    % extracting the translation vector from the rotated point dual
    % quaternion
%     v_r_laj1 = dq_r_laj1.extractTranslation;
%     v_r_laj2 = dq_r_laj1.extractTranslation;
%     v_r_laj3 = dq_r_laj1.extractTranslation;
%     v_r_laj4 = dq_r_laj1.extractTranslation;

    % computing lever joints w.r.t. the error frame using quaternion
    % rotation
    q_r_laj1 = q_error * quaternion([0 dq_base_laj1.extractTranslation]) * q_error.conj;
    q_r_laj2 = q_error * quaternion([0 dq_base_laj2.extractTranslation]) * q_error.conj;
    q_r_laj3 = q_error * quaternion([0 dq_base_laj3.extractTranslation]) * q_error.conj;
    q_r_laj4 = q_error * quaternion([0 dq_base_laj4.extractTranslation]) * q_error.conj;

    % extracting the translation vector from the rotated point quaternion
    v_r_laj1 = q_r_laj1.compact;
    v_r_laj2 = q_r_laj2.compact;
    v_r_laj3 = q_r_laj3.compact;
    v_r_laj4 = q_r_laj4.compact;

    v_r_laj1 = v_r_laj1(2:4);
    v_r_laj2 = v_r_laj2(2:4);
    v_r_laj3 = v_r_laj3(2:4);
    v_r_laj4 = v_r_laj4(2:4);

    % computing error vector for each arms axis
    v_err_r_laj1 = cross(v_r_laj1, dq_error_n);
    v_err_r_laj2 = cross(v_r_laj2, dq_error_n);
    v_err_r_laj3 = cross(v_r_laj3, dq_error_n);
    v_err_r_laj4 = cross(v_r_laj4, dq_error_n);

    % normalizing v_err
    v_err_r_laj1 = v_err_r_laj1 / norm(v_err_r_laj1);
    v_err_r_laj2 = v_err_r_laj2 / norm(v_err_r_laj2);
    v_err_r_laj3 = v_err_r_laj3 / norm(v_err_r_laj3);
    v_err_r_laj4 = v_err_r_laj4 / norm(v_err_r_laj4);

    % modulating accordingly to the theta error
    v_err_r_laj1 = v_err_r_laj1 * dq_error_theta;
    v_err_r_laj2 = v_err_r_laj2 * dq_error_theta;
    v_err_r_laj3 = v_err_r_laj3 * dq_error_theta;
    v_err_r_laj4 = v_err_r_laj4 * dq_error_theta;

    %% Obtaining the error vector projection of each arm unitary x vector

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

    % computing differential position command
    cmd_laj = [ctrl_k * (v_err_r_laj1_sig * norm(v_err_r_laj1_x_proj));...
               ctrl_k * (v_err_r_laj2_sig * norm(v_err_r_laj2_x_proj));...
               ctrl_k * (v_err_r_laj3_sig * norm(v_err_r_laj3_x_proj));...
               ctrl_k * (v_err_r_laj4_sig * norm(v_err_r_laj4_x_proj))];

    % summing to current home position
    cmd_arms_pos = arms_pos_current + cmd_laj;   

    % clips the command if outside predetermined bounds
    cmd_arms_pos_clp = rosi_clip_lever_pos(cmd_arms_pos, la_angle_min, la_angle_max);

    % corrects positions for applying at ROSI with defined signals
    cmd_arms_pos_corr = rosi_correct_joints_signal(cmd_arms_pos_clp.');

    % Only sends the control signal if there is a significant error
    if sum(isnan(cmd_arms_pos_corr)) == 0

        % mounting and publishing ROS msg
        cmd_arms_pos_msg = rosi_mount_arms_pos_msg(cmd_arms_pos_corr, ros_time);
%         send(pub_arms_pos, cmd_arms_pos_msg);

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

%       quiver3(v_laj1(1), v_laj1(2), v_laj1(3), v_err_r_laj1(1), v_err_r_laj1(2), v_err_r_laj1(3),'-k');
%       quiver3(v_laj2(1), v_laj2(2), v_laj2(3), v_err_r_laj2(1), v_err_r_laj2(2), v_err_r_laj2(3),'-k');
%       quiver3(v_laj3(1), v_laj3(2), v_laj3(3), v_err_r_laj3(1), v_err_r_laj3(2), v_err_r_laj3(3),'-k');
%       quiver3(v_laj4(1), v_laj4(2), v_laj4(3), v_err_r_laj4(1), v_err_r_laj4(2), v_err_r_laj4(3),'-k');

      quiver3(v_laj1(1), v_laj1(2), v_laj1(3), v_err_r_laj1_x_proj(1), v_err_r_laj1_x_proj(2), v_err_r_laj1_x_proj(3),'-k');
      quiver3(v_laj2(1), v_laj2(2), v_laj2(3), v_err_r_laj2_x_proj(1), v_err_r_laj2_x_proj(2), v_err_r_laj2_x_proj(3),'-k');
      quiver3(v_laj3(1), v_laj3(2), v_laj3(3), v_err_r_laj3_x_proj(1), v_err_r_laj3_x_proj(2), v_err_r_laj3_x_proj(3),'-k');
      quiver3(v_laj4(1), v_laj4(2), v_laj4(3), v_err_r_laj4_x_proj(1), v_err_r_laj4_x_proj(2), v_err_r_laj4_x_proj(3),'-k');


      hold off;

      
    %% Appending log variables
    
    % appending time
    log_time(1, end+1) = ros_time.Sec;
    log_time(2, end) = ros_time.Nsec;
    
    % current angular position signal
    log_laj_pos(:, end+1) = rosi_correct_joints_signal(data_arms_pos.JointVar);
    
    % error theta
    log_error_theta(end+1) = dq_error_theta;
    
    
    if sum(isnan(cmd_arms_pos_corr)) == 0 % there is significant error signal
        
        % arms projected command signals
        log_laj_delta_sig(:, end+1) = cmd_laj;
        
        % arms command signal
        log_laj_cmd_pos(:, end+1) = cmd_arms_pos_corr;
        
    else % there is no significant error signal
        
        % arms projected command signals
        log_laj_delta_sig(:, end+1) = [0;0;0;0];
        
        % arms command signal
        log_laj_cmd_pos(:, end+1) =  log_laj_cmd_pos(:, end);
    end
    
end















