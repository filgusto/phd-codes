function dq_error = get_attitude_error(dq_ori_world_base, dq_ori_sp, dq_base_director, dq_base_rotating_axis)
% Given current dual quaternion pose, computes the error metrics
% It removes cancels the yaw component
% Input:
%   -dq_ori_world_base: dual quaternion containing current robot orientation
%   -dq_ori_sp: dual quaternion orientation setpoint
%   -dq_base_director: base director vector. One considers the 'pointing
%       forward' reference frame vector
%   -dq_base_rotating_axis: the world frame yaw component.

    % find x-aligned dq_pose
    dq_x_align = get_dq_frame_x_aligned(dq_ori_world_base,...
                                        dq_base_director, ...
                                        dq_base_rotating_axis);
    
    % computing pose error w.r.t. x-aligned frame
    dq_error = dq_x_align.conj *  dq_ori_world_base;

end

