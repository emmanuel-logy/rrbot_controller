function dX = rrbot_ode(t,X)
%RRBOT_ODE Summary of this function goes here
%   Detailed explanation goes here

    % Constants from assignment
    g=9.81;
    m1=1; m2=1;
    I1=0.084; I2=0.084;
    l1=1; l2=1; r1=0.45; r2=0.45;
    
    % Set inputs using the control law of state feedback 
    global K;
    u = -K*X;
    u1 = u(1);
    u2 = u(2);
    
    % Setting up the return values
    dX = zeros(4,1);
    X = num2cell(X);
    [q1, q2, q1d, q2d] = deal(X{:});
%     q1 = wrapTo2Pi(q1);
%     q2 = wrapTo2Pi(q2);
    
    % Creating the state space vectors
    dX(1) = q1d;
    dX(2) = q2d;
    dX(3) = (I2*u1 - I2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*q1d^2*r2^3*sin(q2) + l1*m2^2*q2d^2*r2^3*sin(q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) - l1*m2*r2*u2*cos(q2) + 2*l1*m2^2*q1d*q2d*r2^3*sin(q2) + l1^2*m2^2*q1d^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + I2*l1*m2*q1d^2*r2*sin(q2) + I2*l1*m2*q2d^2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*I2*l1*m2*q1d*q2d*r2*sin(q2))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
    dX(4) = -(I2*u1 - I1*u2 - I2*u2 - l1^2*m2*u2 - m1*r1^2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*q1d^2*r2^3*sin(q2) + l1^3*m2^2*q1d^2*r2*sin(q2) + l1*m2^2*q2d^2*r2^3*sin(q2) - g*l1^2*m2^2*r2*sin(q1 + q2) - I1*g*m2*r2*sin(q1 + q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) + l1*m2*r2*u1*cos(q2) - 2*l1*m2*r2*u2*cos(q2) + 2*l1*m2^2*q1d*q2d*r2^3*sin(q2) + 2*l1^2*m2^2*q1d^2*r2^2*cos(q2)*sin(q2) + l1^2*m2^2*q2d^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + g*l1^2*m2^2*r2*cos(q2)*sin(q1) - g*m1*m2*r1^2*r2*sin(q1 + q2) + I1*l1*m2*q1d^2*r2*sin(q2) + I2*l1*m2*q1d^2*r2*sin(q2) + I2*l1*m2*q2d^2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*l1^2*m2^2*q1d*q2d*r2^2*cos(q2)*sin(q2) + l1*m1*m2*q1d^2*r1^2*r2*sin(q2) + 2*I2*l1*m2*q1d*q2d*r2*sin(q2) + g*l1*m1*m2*r1*r2*cos(q2)*sin(q1))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
    % The above equations are obtained from the rrbot_dyn.m script

end

