function pos_norm = q_extract_pos_error_norm(q_in)

    % converting quaternion to vectorial data type
    q_in_com = q_in.compact;
    
    % computing the magnitude and returning
    pos_norm = norm(q_in_com);
    
end

