function dq_out = dq_relative_pose(qo, t)

    % auxiliary
    q1 = quaternion(1, 0, 0, 0);
    q0 = quaternion(0, 0, 0, 0);
    
    % setting the pure orientation dual quaternion
    dqo = DualQuaternion();
    dqo = dqo.setDQFromQuat(qo, q0);
    
    % translation quaternion
    qt = quaternion(0, 0.5*t(1), 0.5*t(2), 0.5*t(3));
    
    % setting the pure translation dual quaternion
    dqt = DualQuaternion();
    dqt = dqt.setDQFromQuat(q1, qt);
    
    % joining both pure transforms (translation followed by rotation
    dq_out = dqt * dqo;
end

