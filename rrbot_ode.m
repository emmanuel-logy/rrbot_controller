function dX = rrbot_ode(t,X)
%RRBOT_ODE Summary of this function goes here
%   Detailed explanation goes here

    %% Setting up the return values
    dX = zeros(4,1);
%     X = num2cell(X);
%     [q1, q2, q1d, q2d] = deal(X{:});
    q1=X(1);    q2=X(2);    q1d=X(3);    q2d=X(4);

    %% Constants from assignment
    g=9.81;
    m1=1; m2=1;
    I1=0.084; I2=0.084;
    l1=1; l2=1; r1=0.45; r2=0.45;

    %% Defining trajectory
    global a    % each col has coefficients for each joint's trajectory
    % Finding joint1 trajectory (q1 and q1d) at given time using cubic polynomial eq
    a_j1 = a(:,1);
    a0=a_j1(1); a1=a_j1(2); a2=a_j1(3); a3=a_j1(4);
    q1_desired = a0 + a1*t + a2*t^2 + a3*t^3;
    q1d_desired = a1 + 2*a2*t + 3*a3*t^2;
    q1dd_desired = 2*a2 + 6*a3*t;
   
    % Finding joint2 trajectory (q2 and q2d) at given time using cubic polynomial eq
    a_j2 = a(:,2);
    a0=a_j2(1); a1=a_j2(2); a2=a_j2(3); a3=a_j2(4);
    q2_desired = a0 + a1*t + a2*t^2 + a3*t^3;
    q2d_desired = a1 + 2*a2*t + 3*a3*t^2;
    q2dd_desired = 2*a2 + 6*a3*t;

    X_desired = [q1_desired;
                 q2_desired;
                 q1d_desired;
                 q2d_desired];

    U_desired = [q1dd_desired
                 q2dd_desired];

    %% Define the feedback linearized control law
    % [a] virtual control input
    global K;
    v = -K*(X - X_desired) + U_desired;
    % [b] Final feedback linearized control law
    M = [m2*l1^2 + 2*m2*cos(q2)*l1*r2 + m1*r1^2 + m2*r2^2 + I1 + I2,        m2*r2^2 + l1*m2*cos(q2)*r2 + I2
         m2*r2^2 + l1*m2*cos(q2)*r2 + I2,                                   m2*r2^2 + I2];
    EOM_Coriolis_term = [-l1*m2*q2d*r2*sin(q2)*(2*q1d + q2d)
                         l1*m2*q1d^2*r2*sin(q2)];
    EOM_gravity_term = [-g*(l1*m2*sin(q1) + m1*r1*sin(q1) + m2*r2*sin(q1 + q2))
                        -g*m2*r2*sin(q1 + q2)];
    u = M*v + EOM_Coriolis_term + EOM_gravity_term;
    u1 = u(1);
    u2 = u(2);
     
    %% Creating the state space vectors
    dX(1) = q1d;
    dX(2) = q2d;
    dX(3) = (I2*u1 - I2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*q1d^2*r2^3*sin(q2) + l1*m2^2*q2d^2*r2^3*sin(q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) - l1*m2*r2*u2*cos(q2) + 2*l1*m2^2*q1d*q2d*r2^3*sin(q2) + l1^2*m2^2*q1d^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + I2*l1*m2*q1d^2*r2*sin(q2) + I2*l1*m2*q2d^2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*I2*l1*m2*q1d*q2d*r2*sin(q2))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
    dX(4) = -(I2*u1 - I1*u2 - I2*u2 - l1^2*m2*u2 - m1*r1^2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*q1d^2*r2^3*sin(q2) + l1^3*m2^2*q1d^2*r2*sin(q2) + l1*m2^2*q2d^2*r2^3*sin(q2) - g*l1^2*m2^2*r2*sin(q1 + q2) - I1*g*m2*r2*sin(q1 + q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) + l1*m2*r2*u1*cos(q2) - 2*l1*m2*r2*u2*cos(q2) + 2*l1*m2^2*q1d*q2d*r2^3*sin(q2) + 2*l1^2*m2^2*q1d^2*r2^2*cos(q2)*sin(q2) + l1^2*m2^2*q2d^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + g*l1^2*m2^2*r2*cos(q2)*sin(q1) - g*m1*m2*r1^2*r2*sin(q1 + q2) + I1*l1*m2*q1d^2*r2*sin(q2) + I2*l1*m2*q1d^2*r2*sin(q2) + I2*l1*m2*q2d^2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*l1^2*m2^2*q1d*q2d*r2^2*cos(q2)*sin(q2) + l1*m1*m2*q1d^2*r1^2*r2*sin(q2) + 2*I2*l1*m2*q1d*q2d*r2*sin(q2) + g*l1*m1*m2*r1*r2*cos(q2)*sin(q1))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
    % The above equations are obtained from the rrbot_dyn.m script

end

