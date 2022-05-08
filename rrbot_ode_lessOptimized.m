% Less optimized because of using symbolic math... So will go with prof's
% method of copy pasting the equations

function dX = rrbot_ode(t,X)
%RRBOT_ODE Summary of this function goes here
%   Detailed explanation goes here

    % Constants from assignment
%     g=9.81;
%     m1=1; m2=1;
%     I1=0.084; I2=0.084;
%     l1=1; l2=1; r1=0.45; r2=0.45;
    %% Symoblic variables 
    % To avoid copy pasting full equation
    syms m1 m2 I1 I2 r1 r2 l1 l2 'real'     % Robot diemensions
    % Controls related variables
    syms q1 q2 'real'                       % STEP1: Generalized coordinates
    syms u1 u2 'real'                       % STEP2: Generalized inputs
    syms g 'real'                           % Other forces acting on the system             
    syms q1d q2d q1dd q2dd 'real'           % For state space rep

    %% Defining trajectory
    q0 = [180;  90];    q0 = deg2rad(q0);  % denotes q_init of joint1 and joint2
    qf = [0;    0];     qf = deg2rad(qf);
    qd0 = [0;    0];    qd0 = deg2rad(qd0);
    qdf = [0;    0];    qdf = deg2rad(qdf);
    t0 = 0;
    tf = 10;

    % Finding joint1 trajectory (q1 and q1d) at given time using cubic polynomial eq
    a = traj_cubic_solve(q0(1), qf(1), qd0(1), qdf(1), t0, tf); % Trajectory Co-efficients for joint1
    a0=a(1); a1=a(2); a2=a(3); a3=a(4);
    q1_desired = a0 + a1*t + a2*t^2 + a3*t^3;
    q1d_desired = a1 + 2*a2*t + 3*a3*t^2;
   
    % Finding joint2 trajectory (q2 and q2d) at given time using cubic polynomial eq
    a = traj_cubic_solve(q0(2), qf(2), qd0(2), qdf(2), t0, tf); % Trajectory Co-efficients for joint2
    a0=a(1); a1=a(2); a2=a(3); a3=a(4);
    q2_desired = a0 + a1*t + a2*t^2 + a3*t^3;
    q2d_desired = a1 + 2*a2*t + 3*a3*t^2;
       
    X_desired = [q1_desired;
                 q1d_desired;
                 q2_desired;
                 q2d_desired];


    % Define the feedback linearized control law
    % [a] virtual control input
    global K;
    v = -K*(X - X_desired);
    % [b] Final feedback linearized control law
    global M EOM_Coriolis_term EOM_gravity_term
    u = M*v + EOM_Coriolis_term + EOM_gravity_term;
%     syms q1 q2 q1d q2d 'real'           % For state space rep
    u = subs(u, [g,m1,m2,I1,I2,l1,l2,r1,r2,q1,q2,q1d,q2d], [9.81,1,1,0.084,0.084,1,1,0.45,0.45,X(1),X(2),X(3),X(4)]);
    u = double(u);  % To get numeric value from symbolic 
   
    % Setting up the return values
    dX = zeros(4,1);
%     X = num2cell(X);
%     [q1, q2, q1d, q2d] = deal(X{:});

    
    % Creating the state space vectors
    global Xd
    Xd = subs(Xd, [g,m1,m2,I1,I2,l1,l2,r1,r2,q1,q2,q1d,q2d,u1,u2], [9.81,1,1,0.084,0.084,1,1,0.45,0.45,X(1),X(2),X(3),X(4),u(1),u(2)]);
    dX = double(Xd);
%     dX(1) = q1d;
%     dX(2) = q2d;
%     dX(3) = (I2*u1 - I2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*q1d^2*r2^3*sin(q2) + l1*m2^2*q2d^2*r2^3*sin(q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) - l1*m2*r2*u2*cos(q2) + 2*l1*m2^2*q1d*q2d*r2^3*sin(q2) + l1^2*m2^2*q1d^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + I2*l1*m2*q1d^2*r2*sin(q2) + I2*l1*m2*q2d^2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*I2*l1*m2*q1d*q2d*r2*sin(q2))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
%     dX(4) = -(I2*u1 - I1*u2 - I2*u2 - l1^2*m2*u2 - m1*r1^2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*q1d^2*r2^3*sin(q2) + l1^3*m2^2*q1d^2*r2*sin(q2) + l1*m2^2*q2d^2*r2^3*sin(q2) - g*l1^2*m2^2*r2*sin(q1 + q2) - I1*g*m2*r2*sin(q1 + q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) + l1*m2*r2*u1*cos(q2) - 2*l1*m2*r2*u2*cos(q2) + 2*l1*m2^2*q1d*q2d*r2^3*sin(q2) + 2*l1^2*m2^2*q1d^2*r2^2*cos(q2)*sin(q2) + l1^2*m2^2*q2d^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + g*l1^2*m2^2*r2*cos(q2)*sin(q1) - g*m1*m2*r1^2*r2*sin(q1 + q2) + I1*l1*m2*q1d^2*r2*sin(q2) + I2*l1*m2*q1d^2*r2*sin(q2) + I2*l1*m2*q2d^2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*l1^2*m2^2*q1d*q2d*r2^2*cos(q2)*sin(q2) + l1*m1*m2*q1d^2*r1^2*r2*sin(q2) + 2*I2*l1*m2*q1d*q2d*r2*sin(q2) + g*l1*m1*m2*r1*r2*cos(q2)*sin(q1))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
    % The above equations are obtained from the rrbot_dyn.m script

end

