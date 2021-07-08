function [ret_v, ret_sig]= vector_projection(v_in, v_proj)
% Projects v_in on v_proj and returns projected vector and angle signal
    
    % computing projected vector
    ret_v = (dot(v_in, v_proj) / dot(v_proj, v_proj)) * v_proj;
    
    % computing projected signal
    ret_sig = sig(dot(v_in, v_proj));

end

