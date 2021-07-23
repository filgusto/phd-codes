function dq_ret = remove_yaw(dq_in)
% Removes yaw component from an input dual quaternion

    % decomposing rotation component into roll pitch yaw
    rpy = dq_in.extractRotationRPY;
    
    % creating rotation dual quaternion to cancel the yaw component
    dq_rot = DualQuaternion.pureRotation([0 0 -1*rpy(3)]);
    
    % rotating dq_in to cancel the yaw rotation
    dq_ret = dq_in * dq_rot;

end

