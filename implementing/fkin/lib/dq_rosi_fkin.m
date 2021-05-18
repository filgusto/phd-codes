function dq_res = dq_rosi_fkin(q, dq_world_base,dq_base_arm, dq_arm_arr, dq_jwrist_tcp)
    
    % updating joints transforms
    dq_arm_arr_up = {};
    for i = 1:length(dq_arm_arr)
        dq_arm_arr_up{i} = dq_arm_arr{i} * dq_joint_rot(q(i));
    end

    % computing until the manipulator base
    dq_res = DualQuaternion();
    dq_res = dq_res * dq_world_base * dq_base_arm;

    % computing fkin until the EE
    for i=1:length(dq_arm_arr)
        dq_res = dq_res * dq_arm_arr_up{i};
    end

    % from last manipulator joint to the TCP
    dq_res = dq_res * dq_jwrist_tcp;

end

