
clear all;
clc;
close all;

%% preamble

% inform where the phd-codes folder is located for finding related code
folder_phd_codes = '/home/filipe/gitSources/doc/phd-codes';

addpath(strcat(folder_phd_codes,'/implementing/modelling'));
addpath(strcat(folder_phd_codes,'/implementing/modelling/lib'));
addpath(strcat(folder_phd_codes,'/implementing/lib/dq'));
addpath(strcat(folder_phd_codes,'/implementing/fkin/lib'));
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
sub_pos_manipulator_joints = rossubscriber(r_tpc.pos_joints.adr, r_tpc.pos_joints.msg);
sub_vel_manipulator_joints = rossubscriber(r_tpc.vel_joints.adr, r_tpc.vel_joints.msg);

sub_pose_vel_rosi = rossubscriber(r_tpc.pose_vel_rosi.adr, r_tpc.pose_vel_rosi.msg);
sub_pose_vel_j1 = rossubscriber(r_tpc.pose_vel_mani_joint1.adr, r_tpc.pose_vel_mani_joint1.msg);

sub_pose_rosi = rossubscriber(r_tpc.pose_rosi.adr, r_tpc.pose_rosi.msg);
sub_pose_j1 = rossubscriber(r_tpc.pose_mani_joint1.adr, r_tpc.pose_mani_joint1.msg);


%% Loop

disp('Loop initiated...');
while true
    
    %% retrieving variables
    
    % retrieve joints position
    q = ros_retrieve_mani_joints(sub_pos_manipulator_joints);
    
    % retrieve joints velocity
    qd = ros_retrieve_mani_joints(sub_vel_manipulator_joints);
    
    % retrieve base pose
    dq_world_base = ros_retrieve_dq(sub_pose_rosi);
    
    % retrieve j1 pose
    dq_world_j1_gt = ros_retrieve_dq(sub_pose_j1);
    
    % retrieve base vel
    omega_world_base_world = ros_retrieve_dq(sub_pose_vel_rosi);
    
    % retrieve j1 vel
    omega_world_j1_world_gt = ros_retrieve_dq(sub_pose_vel_j1);
    
    
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
   
    
    %% Computing fkin
    
    % variable for the fkin vel
    omega_world_j1_j1_res = DualQuaternion(zeros(8));
    
    %% base -> j1
    
    omega_world_base_base = DualQuaternion.adj(dq_world_base.conj, omega_world_base_world);
    
    res_dq_rest = dq_base_arm * dq_arm_arr_up{1} * dq_arm_arr_up{2};
                
    omega_world_j1_j1_res = omega_world_j1_j1_res + DualQuaternion.adj(res_dq_rest.conj, omega_world_base_base);
        
    
    %% manipulator -> j1
    omega_base_arm_arm = DualQuaternion(zeros(8));
    
    res_dq_rest = dq_arm_arr_up{1} * dq_arm_arr_up{2};
                
    omega_world_j1_j1_res = omega_world_j1_j1_res + DualQuaternion.adj(res_dq_rest.conj, omega_base_arm_arm);

    
    %% joint 0 -> joint 1
    res_dq_rest = dq_arm_arr_up{2};
    
    omega_world_j1_j1_res = omega_world_j1_j1_res + DualQuaternion.adj(res_dq_rest.conj, omega_joint_screw{1});    
  
    
    
    %% Comparing fkin
     
%     dq_world_j1_res = dq_world_base * dq_base_arm * dq_arm_arr_up{1} * dq_arm_arr_up{2};
%                    
%     dq_world_j1_gt
%     dq_world_j1_res
%     
%     comp_dq = dq_world_j1_gt.conj * dq_world_j1_res;
%     comp_dq = comp_dq.rectify
%     
%     plot([DualQuaternion dq_world_j1_gt dq_world_j1_res]);


    %% Comparing linear velocity from gt and res for base frame
    % proving frame change
    
%     omega_world_base_world
%     omega_world_base_base
%     
%     aux_1 = omega_world_base_world.compact;
%     aux_2 = omega_world_base_base.compact;
%     
%     norm_w = norm(aux_1(end-2:end))
%     norm_b = norm(aux_2(end-2:end))


 %% Changing dual velocity frame of gt base
 % Computing dual quaternion kinematics of base
    
%     omega_world_base_world   % comes directly from the simulator
%     omega_world_base_base    % computed by the adjoint operator
%     
%     dqd_world_base_w = 0.5 * omega_world_base_world * dq_world_base;
%     dqd_world_base_w = dqd_world_base_w.rectify
%     
%     dqd_world_base_b = 0.5 * dq_world_base * omega_world_base_base;
%     dqd_world_base_b = dqd_world_base_b.rectify
%     
%     dqd_world_base_w - dqd_world_base_b


    %% Comparing linear velocity from gt and res for j1 frame
    % proving frame change
    
    omega_world_j1_world_gt
    omega_world_j1_j1_gt = DualQuaternion.adj(dq_world_j1_gt.conj, omega_world_j1_world_gt)
    omega_world_j1_j1_res
    
    omega_world_j1_j1_gt - omega_world_j1_j1_res
    
%     aux_1 = omega_world_j1_j1_gt.compact;
%     aux_2 = omega_world_j1_j1_res.compact;
%     
%     norm_gt = norm(aux_1(end-2:end))
%     norm_res = norm(aux_2(end-2:end))
    
%     omega_world_j1_world_gt
%     aux_3 = omega_world_j1_world_gt.compact;
%     norm_dq_p = norm(aux_3(1:4))
%     norm_dq_d = norm(aux_3(end-3:end))
%     
%     omega_world_j1_j1_gt
%     aux_4 = omega_world_j1_j1_gt.compact;
%     norm_dq_p = norm(aux_4(1:4))
%     norm_dq_d = norm(aux_4(end-3:end))
   
    
%% Computing dual quaternion kinematics of tcp

% %     omega_world_tcp_world_gt   % comes directly from the simulator
%     omega_world_tcp_tcp_gt = DualQuaternion.adj(dq_world_tcp_gt.conj, omega_world_tcp_world_gt) % computed from gt
%     omega_world_tcp_tcp_res    % computed by the fkin method
%     
%     
%     % computing dqd world tcp world from ground truth
% %     dqd_world_tcp_world_gt = 0.5 * omega_world_tcp_world_gt * dq_world_tcp_gt;
% %     dqd_world_tcp_world_gt = dqd_world_tcp_world_gt.rectify
% 
%     % computing dqd world tcp tcp from ground truth
%     dqd_world_tcp_tcp_gt = 0.5 * dq_world_tcp_gt * omega_world_tcp_tcp_gt;
%     dqd_world_tcp_tcp_gt = dqd_world_tcp_tcp_gt.rectify
%     
%     % dqd world tcp tcp from fkin
%     dqd_world_tcp_tcp_res = 0.5 * dq_world_tcp_gt * omega_world_tcp_tcp_res;
%     dqd_world_tcp_tcp_res = dqd_world_tcp_tcp_res.rectify
%     
%     % comparing gt
% %     dqd_world_tcp_world_gt - dqd_world_tcp_tcp_gt
%     
%     % comparing computed
%     dqd_world_tcp_tcp_gt - dqd_world_tcp_tcp_res

    %% Changing dual velocity frame of res tcp
    % Failing
    
%     omega_world_tcp_tcp_res
%     res_omega_world_tcp_world = DualQuaternion.adj(dq_world_tcp_gt, omega_world_tcp_tcp_res)
% 
%     aux_1 = omega_world_tcp_tcp_res.compact;
%     aux_2 = res_omega_world_tcp_world.compact;
%     
%     norm_gt = norm(aux_1(end-2:end))
%     norm_res = norm(aux_2(end-2:end))

    %% Comparing dq_dot
    % Failing
    
%     % dqd computed by the gt method
%     dqd_gt = 0.5 * omega_world_tcp_world_gt * dq_world_tcp_gt;
%     dqd_gt = dqd_gt.rectify
%     
%     % dqd computed by the fkin differential method
%     dqd_res = 0.5 * dq_world_tcp_gt * res_omega;
%     dqd_res = dqd_res.rectify
%     
%     comp_sub = dqd_gt - dqd_res    
%     
%     comp_mul = dqd_gt.conj * dqd_res


              
end



