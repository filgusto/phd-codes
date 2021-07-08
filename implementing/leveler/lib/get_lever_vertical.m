function v_x_unit_rot = get_lever_vertical(v_vertical, dq_r_laj)

    % lever arm vertical vector axis
    x_unit = DualQuaternion.pureTranslation(v_vertical);
    
    % creating lever arm rotation dual quaternion
    dq_rot = DualQuaternion(dq_r_laj.q_p, quaternion([0 0 0 0]));
    
    % computing the rotated vertical vector
    x_unit_rot = dq_rot * x_unit * dq_rot.conj;
    
    % extracting to return its vector
    v_x_unit_rot = x_unit_rot.extractTranslation;

end

