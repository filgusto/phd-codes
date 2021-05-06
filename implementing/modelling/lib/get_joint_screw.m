function dq_screw = get_joint_screw(type, axis)
% returns a joint screw dual quaternion considering its type (revolute or
% prismatic) and actuation axis.

    % descovering which dq strucute this screw is
    switch true
        case type == 'r' & axis == 'x'
            p = [0, 1, 0, 0];
            d = [0, 0, 0, 0];
        case type == 'r' & axis == 'y'
            p = [0, 0, 1, 0];
            d = [0, 0, 0, 0];
        case type == 'r' & axis == 'z'
            p = [0, 0, 0, 1];
            d = [0, 0, 0, 0];
        case type == 'p' & axis == 'x'
            p = [0, 0, 0, 0];
            d = [0, 1, 0, 0];
        case type == 'p' & axis == 'y'
            p = [0, 0, 0, 0];
            d = [0, 0, 1, 0];
        case type == 'p' & axis == 'z'
            p = [0, 0, 0, 0];
            d = [0, 0, 0, 1];
               
    end
    
    % mounting and returning the dual quaternion
    q_p = quaternion(p);
    q_d = quaternion(d);
    dq_screw = DualQuaternion(q_p, q_d);
        
end

