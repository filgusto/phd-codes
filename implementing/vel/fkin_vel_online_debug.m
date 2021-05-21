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
sub_pose_vel_rosi = rossubscriber(r_tpc.pose_vel_rosi.adr, r_tpc.pose_vel_rosi.msg);
sub_pose_vel_mani = rossubscriber(r_tpc.pose_vel_mani.adr, r_tpc.pose_vel_mani.msg);
sub_pose_vel_tcp = rossubscriber(r_tpc.pose_vel_tcp.adr, r_tpc.pose_vel_tcp.msg);
sub_pose_rosi = rossubscriber(r_tpc.pose_rosi.adr, r_tpc.pose_rosi.msg);
sub_pose_tcp = rossubscriber(r_tpc.pose_tcp.adr, r_tpc.pose_tcp.msg);
sub_pos_manipulator_joints = rossubscriber(r_tpc.pos_joints.adr, r_tpc.pos_joints.msg);
sub_vel_manipulator_joints = rossubscriber(r_tpc.vel_joints.adr, r_tpc.vel_joints.msg);

sub_pose_vel_mani_joint = rossubscriber(r_tpc.pose_vel_mani_joint2.adr, r_tpc.pose_vel_mani_joint2.msg);

%% initializing variables

% the manipulator is still w.r.t. robot
omega_base_manipulator_manipulator = DualQuaternion(zeros(8));

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
    
    % tcp pose
    %dq_world_tcp = ros_retrieve_dq(sub_pose_tcp);
    
    % retrieve base velocity
    omega_world_base_world = ros_retrieve_dq(sub_pose_vel_rosi);
    
    % retrieve manipulator base velocity
    %omega_world_arm_world = ros_retrieve_dq(sub_pose_vel_mani);
    
    % retrieve manipulator joint 0 velocity
    %omega_world_j_world = ros_retrieve_dq(sub_pose_vel_mani_joint);
    
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
   
    % variable for the fkin vel
    res_omega = DualQuaternion(zeros(8));
    
    %% Computing fkin
    
    %% world -> base    
    res_dq = dq_world_base;
    omega_world_base_base = DualQuaternion.adj(res_dq.conj, omega_world_base_world);
    
    res_dq_rest = dq_base_arm * dq_arm_arr_up{1} * dq_arm_arr_up{2} * dq_arm_arr_up{3} * ...
                  dq_arm_arr_up{4} * dq_arm_arr_up{5} * dq_arm_arr_up{6} * ...
                  dq_arm_arr_up{7} * dq_j6_tcp;
                
    res_omega = res_omega + DualQuaternion.adj(res_dq_rest.conj, omega_world_base_base);
        
    
    %% base -> manipulator    
    omega_base_arm_tcp = DualQuaternion(zeros(8));
    
    res_dq_rest = dq_arm_arr_up{1} * dq_arm_arr_up{2} * dq_arm_arr_up{3} * ...
                  dq_arm_arr_up{4} * dq_arm_arr_up{5} * dq_arm_arr_up{6} * ...
                  dq_arm_arr_up{7} * dq_j6_tcp;
                
    res_omega = res_omega + DualQuaternion.adj(res_dq_rest.conj, omega_base_arm_tcp);
    
    
    %% manipulator -> joint 0
    res_dq_rest = dq_arm_arr_up{2} * dq_arm_arr_up{3} * ...
                  dq_arm_arr_up{4} * dq_arm_arr_up{5} * dq_arm_arr_up{6} * ...
                  dq_arm_arr_up{7} * dq_j6_tcp;
    
    res_omega = res_omega + DualQuaternion.adj(res_dq_rest.conj, omega_joint_screw{1});
   

    %% joint 0 -> joint 1
    res_dq_rest = dq_arm_arr_up{3} * ...
                   dq_arm_arr_up{4} * dq_arm_arr_up{5} * dq_arm_arr_up{6} * ...
                   dq_arm_arr_up{7} * dq_j6_tcp;
    
    res_omega = res_omega + DualQuaternion.adj(res_dq_rest.conj, omega_joint_screw{2});
    
    
    %% joint 1 -> joint 2
    res_dq_rest =  dq_arm_arr_up{4} * dq_arm_arr_up{5} * dq_arm_arr_up{6} * ...
                   dq_arm_arr_up{7} * dq_j6_tcp;
    
    res_omega = res_omega + DualQuaternion.adj(res_dq_rest.conj, omega_joint_screw{3});
    
    
    %% joint 3 -> joint 4
    res_dq_rest =  dq_arm_arr_up{5} * dq_arm_arr_up{6} * ...
                   dq_arm_arr_up{7} * dq_j6_tcp;
    
    res_omega = res_omega + DualQuaternion.adj(res_dq_rest.conj, omega_joint_screw{4});
    
    
    %% joint 4 -> joint 5
    res_dq_rest =  dq_arm_arr_up{6} * ...
                   dq_arm_arr_up{7} * dq_j6_tcp;
    
    res_omega = res_omega + DualQuaternion.adj(res_dq_rest.conj, omega_joint_screw{5});
    
    
    %% joint 5 -> joint 6
    res_dq_rest = dq_arm_arr_up{7} * dq_j6_tcp;
    
    res_omega = res_omega + DualQuaternion.adj(res_dq_rest.conj, omega_joint_screw{6});
    
    
    %% joint 6 -> tcp
    res_dq_rest = dq_j6_tcp;
    
    res_omega = res_omega + DualQuaternion.adj(res_dq_rest.conj, omega_joint_screw{7})
    
    
    %% GT
    res_dq_rest = dq_world_base * dq_base_arm * dq_arm_arr_up{1} * dq_arm_arr_up{2} * dq_arm_arr_up{3} * ...
                  dq_arm_arr_up{4} * dq_arm_arr_up{5} * dq_arm_arr_up{6} * ...
                  dq_arm_arr_up{7} * dq_j6_tcp;
    
    
    omega_world_tcp_tcp = DualQuaternion.adj(res_dq_rest.conj, omega_world_tcp_world)
    omega_world_tcp_world
    
    
    res_omega -  omega_world_tcp_tcp
    
%     % joint 0 -> joint 1
%     res_dq_rest = dq_arm_arr_up{3};
%     res_omega = res_omega + DualQuaternion.adj(res_dq_rest.conj, omega_joint_screw{2});
%     
%     
%     % joint 1 -> joint 2
%     res_omega = res_omega + omega_joint_screw{3}
%     
    
    %% Debug
%     res_dq = dq_world_base * dq_base_arm * dq_arm_arr_up{1} *  dq_arm_arr_up{2} * dq_arm_arr_up{3};
%     omega_world_j2_j2_gt = DualQuaternion.adj(res_dq.conj, omega_world_j_world)
   
    
    
    
    
    %for i=1:lenght(dq_arm_arr)
        
        
    %end
%     
%     for i=length(dq_arm_arr):-1:1
%         
%         % computing the fkin step-by-step from the last joint
%         if i ~= length(dq_arm_arr)
%             res_dq = dq_arm_arr_up{i+1} * res_dq;
%         else
%             res_dq = dq_j6_tcp * res_dq;
%         end
%         
%         % computing adjoint and summing up to accumulated result
%         res_omega = res_omega + DualQuaternion.adj(res_dq.conj, omega_joint_screw{i});
%         
%     end
%     
%     % summing for the joint manipulator w.r.t. robot base
%     res_dq = dq_arm_arr_up{1} * res_dq;
%     res_omega = res_omega + DualQuaternion.adj(res_dq.conj, omega_base_manipulator_manipulator);
%     
%     % summing for the robot base w.r.t. inertial reference
%     omega_world_base_base = DualQuaternion.adj(dq_world_base.conj, omega_world_base_world);
%     res_dq = dq_base_arm * res_dq;
%     res_omega = res_omega + DualQuaternion.adj(res_dq.conj, omega_world_base_base)
%     
%     % finishing computing the fkin
%     res_dq = dq_world_base * res_dq;
    
%     %% computing rosi fkin
%   
%     % computing omega tcp w.r.t. world expressed in tcp
%     omega_world_tcp_tcp = DualQuaternion.adj(res_dq.conj, omega_world_tcp_world)
%     
%     res_omega - omega_world_tcp_tcp
%        
%     %aux_res = res_omega_world_tcp_tcp.q_d.compact;
%     %aux_gt = omega_world_tcp_tcp.q_d.compact;
% 
%     % computing the dual pose derivative from the simulator gt
%     %dqd_world_tcp = 0.5 *  omega_world_tcp_world * res_dq_world_tcp
%     
%     % comparing the dual pose derivative from the simulator gt and computed
%     % omega
%     %dqd_world_tcp_gt = 0.5 * res_dq_world_tcp * omega_world_tcp_tcp
%     %dqd_world_tcp_res = 0.5 * res_dq_world_tcp*  res_omega_world_tcp_tcp
%     
%     %dqd_world_tcp_gt - dqd_world_tcp_res
%     
%     %aux2_owtt - aux2_owtw
%     
%     %res_omega_world_tcp_tcp - aux_omega
%     
%     %aux_omega = omega_world_tcp_tcp;
%     
%     %res_omega_world_tcp_tcp;
%     
%     % computing dqd
%     %dqd_world_base = 0.5 * omega_world_base * dq_world_base;    
end



