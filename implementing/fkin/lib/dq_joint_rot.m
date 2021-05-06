function r = dq_joint_rot(angle, axis)

    if nargin < 2 
        % default rotation axis is around z axis
        axis = [0, 0, 1]; 
    end

   % creates and return a pure rotation dual quaternion   
   r = DualQuaternion.pureRotation(angle, axis);

end

