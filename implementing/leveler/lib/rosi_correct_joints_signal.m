function ret = rosi_correct_joints_signal(v_in)
%ROSI_CORRECT_JOINTS_SIGNAL For arms and wheels joints

    ret = v_in .* [-1; 1; 1; -1];
end

