function q = ros_retrieve_joints(sub_pose, sub_mani)

    % collects ros_messages
    topic_data_pose = receive(sub_pose, 3);
    topic_data_mani = receive(sub_mani, 3);

    % extracting pose values
    p_x = topic_data_pose.Pose.Position.X;
    p_y = topic_data_pose.Pose.Position.X;
    ori_q = quaternion(topic_data_pose.Pose.Orientation.W,...
                       topic_data_pose.Pose.Orientation.X,...
                       topic_data_pose.Pose.Orientation.Y,...
                       topic_data_pose.Pose.Orientation.Z);
    ori_eul = quat2eul(ori_q,'ZYX');
    phi_z = ori_eul(1);
    
    % extracting manipulator data
    q_mani = topic_data_mani.JointVariable;

    % stacking states
    q = [p_x; p_y; phi_z; q_mani];

end

