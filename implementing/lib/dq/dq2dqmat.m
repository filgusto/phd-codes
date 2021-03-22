% Converts dual quaternion components in DQ library quaternion format
function out = dq2dqmat(in)

    % for using dq library namespace
    include_namespace_dq;

    % mounting the dual quaternion
    out = (in.Wp + in.Xp*i_ + in.Yp*j_ + in.Zp*k_) + ...
          (in.Wd + in.Xd*i_ + in.Yd*j_ + in.Zd*k_) * E_;

end