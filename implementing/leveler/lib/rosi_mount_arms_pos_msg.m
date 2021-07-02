function ret_msg = rosi_mount_arms_pos_msg(pos_vec, ros_time)
    
    % mounting and returning ROS message
    ret_msg = rosmessage('sim_rosi/RosiMovementHeader');
    ret_msg.Header.Stamp = ros_time;
    ret_msg.Header.FrameId = 'lever_arms';
    ret_msg.NodeID = [1, 2, 3, 4];
    ret_msg.JointVar = pos_vec;
    
end

