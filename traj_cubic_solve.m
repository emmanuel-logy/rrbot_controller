function a = traj_cubic_solve(q0, qf, qd0, qdf, t0, tf)
%TRAJ_GEN_CUBIC Summary of this function goes here
%   Detailed explanation goes here
    
    % A*a = b... From lecture, we know A and b.. solving for co-efficients
    A = [1      t0      t0^2    t0^3;
        0       1       2*t0    3*t0^2;
        1       tf      tf^2    tf^3;
        0       1       2*tf    3*tf^2];

    b = [q0     qd0     qf      qdf]';

    a = A\b;    % equivalent to inv(A)*b
end

