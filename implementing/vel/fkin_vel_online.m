clear all;
clc;
close all;

%% preamble

% inform where the phd-codes folder is located for finding related code
folder_phd_codes = '/home/filipe/gitSources/doc/phd-codes';

addpath(strcat(folder_phd_codes,'/implementing/modelling'));
addpath(strcat(folder_phd_codes,'/implementing/modelling/lib'));
addpath(strcat(folder_phd_codes,'/implementing/lib/dq'));
addpath('./lib');

%% preamble

% loading robot dualquaternion modules
load(strcat(folder_phd_codes,'/implementing/modelling/','model_dq.mat'));


%% ROS

% initializing ros connection
ros_gentle_init();

% loading ros rosi topics
r_tpc = rosi_ros_topic_info();

% subscribing
sub_pose_vel_rosi = rossubscriber(r_tpc.pose_vel_rosi.adr, r_tpc.pose_vel_rosi.msg);
sub_pose_vel_tcp = rossubscriber(r_tpc.pose_vel_tcp.adr, r_tpc.pose_vel_tcp.msg);
sub_pose_rosi = rossubscriber(r_tpc.pose_rosi.adr, r_tpc.pose_rosi.msg);
sub_pos_manipulator_joints = rossubscriber(r_tpc.pos_joints.adr, r_tpc.pos_joints.msg);
sub_vel_manipulator_joints = rossubscriber(r_tpc.vel_joints.adr, r_tpc.vel_joints.msg);

%% initializing variables

% the manipulator is still w.r.t. robot
omega_base_manipulator = DualQuaternion(zeros(8));

aux_omega = DualQuaternion();

%% Loop

disp('Loop initiated...');
while true
    
    %% retrieving variables
    
    % retrieve joints position
    q = ros_retrieve_mani_joints(sub_pos_manipulator_joints);
    
    % retrieve joints velocity
    qd = ros_retrieve_mani_joints(sub_vel_manipulator_joints);
    
    % retrieve rosi base pose
    dq_world_base = ros_retrieve_dq(sub_pose_rosi);
    
    % retrieve base velocity
    omega_world_base = ros_retrieve_dq(sub_pose_vel_rosi);
    
    % retrieve tcp velocity
    omega_world_tcp_world = ros_retrieve_dq(sub_pose_vel_tcp);
   
    
    %% computations
    
    % updating joints screw twists with joint velocities and relative
    % transforms
    dq_arm_arr_up = {};
    omega_joint_screw = {};
    for i=1:length(dq_arm_arr)
        
        % updating relative transform considering joints position
        dq_arm_arr_up{i} = dq_arm_arr{i} * dq_joint_rot(q(i));
        
        % updating joint screws given joints velocities
        omega_joint_screw{i} = get_joint_screw('r', 'z') * qd(i);
    end
    
    % variable for the fkin
    res_dq_fkin = DualQuaternion();
   
    % variable for the fkin vel
    res_omega_world_tcp_tcp = DualQuaternion(zeros(8));
    
    for i=length(dq_arm_arr):-1:1
        
        % computing the fkin step-by-step from the last joint
        if i ~= length(dq_arm_arr)
            res_dq_fkin = dq_arm_arr_up{i+1} * res_dq_fkin;
        else
            res_dq_fkin = dq_j6_tcp * res_dq_fkin;
        end
        
        % computing adjoint and summing up to accumulated result
        res_omega_world_tcp_tcp = res_omega_world_tcp_tcp + DualQuaternion.adj(res_dq_fkin.conj, omega_joint_screw{i});
        
    end
    
    % summing for the joint manipulator w.r.t. robot base
    res_dq_fkin = dq_arm_arr_up{1} * res_dq_fkin;
    res_omega_world_tcp_tcp = res_omega_world_tcp_tcp + DualQuaternion.adj(res_dq_fkin.conj, omega_base_manipulator);
    
    % summing for the robot base w.r.t. inertial reference
    res_dq_fkin = dq_base_arm * res_dq_fkin;
    res_omega_world_tcp_tcp = res_omega_world_tcp_tcp + DualQuaternion.adj(res_dq_fkin.conj, omega_world_base)
    
    % finishing computing the fkin
    res_dq_fkin = dq_world_base * res_dq_fkin;
    
    %% computing rosi fkin
  
    % computing omega tcp w.r.t. world expressed in tcp
    omega_world_tcp_tcp = DualQuaternion.adj(res_dq_fkin.conj, omega_world_tcp_world)
    
    % PAREI AQUI PAREI AQUI
    res_omega_world_tcp_tcp - aux_omega
    
    aux_omega = omega_world_tcp_tcp;
    
    %res_omega_world_tcp_tcp;
    
    % computing dqd
    %dqd_world_base = 0.5 * omega_world_base * dq_world_base;    
end



















