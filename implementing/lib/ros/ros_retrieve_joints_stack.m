function q = ros_retrieve_joints_stack(sub_base, sub_mani)

    % collects ros_messages
    topic_data_pose = receive(sub_base, 3);
    topic_data_mani = receive(sub_mani, 3);

    % extracting pose values
    p_x = topic_data_pose.Xd;
    p_y = topic_data_pose.Yd;
    ori_z = quaternion(topic_data_pose.Wp,...
                       topic_data_pose.Xp,...
                       topic_data_pose.Yp,...
                       topic_data_pose.Zp);
    ori_eul = quat2eul(ori_z,'ZYX');
    phi_z = ori_eul(1);
    
    % extracting manipulator data
    q_mani = topic_data_mani.JointVariable;

    % stacking states
    q = [p_x; p_y; phi_z; q_mani];

end

