clear all; close all; clc;

%% Symoblic variables

% Defining the robot
syms m1 m2 I1 I2 r1 r2 l1 l2 'real'     % Robot diemensions

% Controls related variables
syms q1 q2 'real'                       % STEP1: Generalized coordinates
syms u1 u2 'real'                       % STEP2: Generalized inputs
syms g 'real'                           % Other forces acting on the system             
syms q1d q2d q1dd q2dd 'real'           % For state space rep


%% STEP1: Generalized coordinates
q = [q1; q2];
qd = [q1d; q2d];
qdd = [q1dd; q2dd];

% STEP1: Generalized inputs
u = [u1; u2];

%% Step3: Lagrangian Derivation
% Kinetic Energy of link1
x1 = r1*sin(q1);
y1 = r1*cos(q1);
x1d = r1*q1d*cos(q1);
y1d = -r1*q1d*sin(q1);
K1 = 0.5*m1*(x1d^2 + y1d^2) + 0.5*I1*q1d^2;

% Kinetic Energy of link2
x2 = l1*sin(q1) + r2*sin(q1+q2);
y2 = l1*cos(q1) + r2*cos(q1+q2);
x2d = l1*q1d*cos(q1) + r2*(q1d+q2d)*cos(q1+q2);
y2d = -l1*q1d*sin(q1) - r2*(q1d+q2d)*sin(q1+q2);
K2 = 0.5*m2*(x2d^2 + y2d^2) + 0.5*I2*(q1d+q2d)^2;

% Total Kinetic Energy
KE = K1 + K2;

% Potential Energy of link1
P1 = m1*g*(y1);             % y1 is the height of m1 COM from the ground

% Potential Energy of link2
P2 = m2*g*(y2);             % y2 is the height of m2 COM from the ground

% Total Potential Energy
PE = P1 + P2;

% Lagrangian Function
L = KE - PE;


%% STEP4a: Euler-Lagrangian Equations of Motion
% EOM
DL_Dq = jacobian(L, q);  % D denotes partial diff and d is differentiation
DL_Dqd = jacobian(L, qd);
dDL_dtDqd = jacobian(DL_Dqd, [q; qd]) * [qd; qdd];          % Chain Rule
EOM = dDL_dtDqd - DL_Dq' - u;
EOM = simplify(EOM);
display(EOM);


%% STEP4b: Simplify to get it in manipulator form:  M*qdd + C*qd + g = tau
% Finding g(q) term... by setting qd and qdd to zero
global EOM_gravity_term;    % to use in ode function
EOM_gravity_term = subs(EOM, [q1d, q1dd, q2d, q2dd, u1, u2], [0,0,0,0,0,0]);
EOM_gravity_term = simplify(EOM_gravity_term);
display(EOM_gravity_term);

% Finding M(q) term... by setting qd to zero and subtracting g(q) from that eqaution
EOM_Mass_term = subs(EOM, [q1d, q2d, u1, u2], [0,0,0,0]);
EOM_Mass_term = EOM_Mass_term - EOM_gravity_term;
EOM_Mass_term = simplify(EOM_Mass_term);
display(EOM_Mass_term);
% Mass matrix is n*n for n-DOF robot.. we need to separate out those q1dd
% and q2dd terms and generate the mass matrix
global M;   
M = zeros(2,2);
M = jacobian(EOM_Mass_term, qdd);
M = simplify(M);
display(M);

% Finding C(q,qd) term... by subtracting M and g terms
global EOM_Coriolis_term;
EOM_Coriolis_term = subs(EOM, [u1, u2], [0,0]);
EOM_Coriolis_term = EOM_Coriolis_term - EOM_Mass_term - EOM_gravity_term;
EOM_Coriolis_term = simplify(EOM_Coriolis_term);
display(EOM_Coriolis_term);

% Finding tau term... by setting q,qd,qdd to zero..except for u
EOM_tau_term = subs(EOM, [q1, q2, q1d, q1dd, q2d, q2dd], [0,0,0,0,0,0]);
EOM_tau_term = 0 - EOM_tau_term;        % taking to RHS
EOM_tau_term = simplify(EOM_tau_term);
display(EOM_tau_term);

% Write in manipulator form and verify if we get back same EOM
EOM_manipForm = M*qdd + EOM_Coriolis_term + EOM_gravity_term - EOM_tau_term;    % M*qdd = EOM_Mass_term
EOM_manipForm = simplify(EOM_manipForm);
display(EOM_manipForm);
% isequal(EOM,EOM_manipForm)    % some terms are taken common out.. hence
% not getting able to verify in code, but verified manually!!


%% STEP5: State Space Representation
% State vector
X = sym ('X', [4, 1]); 
X(1) = q1;
X(2) = q2;
X(3) = q1d;
X(4) = q2d;

eq1 = EOM(1);
eq2 = EOM(2);
sol = solve([eq1==0, eq2==0], [q1dd, q2dd]);
display(sol.q1dd);
display(sol.q2dd);

% State Space Form
global Xd       % To use it in ODE function later
Xd = [X(3); X(4); sol.q1dd; sol.q2dd];
disp("-----State Space Representation-----");
display(Xd);

% For grouping of terms
% simplify(sol.q1dd)
% simplify(sol.q2dd)


%% STEP6: Feedack Linearized Control Law
% [1] For virtual input v = qdd (double integrator), let's design a
% state-feedback controller

% This is correct, if my states were X = [q1,q1d,q2,q2d]'
% A = [0  1;
%      0  0];         % For 1 joint
% A = blkdiag(A, A);  % For 2 joints
% B = [0;
%      1];            % For 1 joint
% B = blkdiag(B, B);  % For 2 joints

% The states I defined on top is X = [q1,q2,q1d,q2d]'
A = [0 0 1 0;
     0 0 0 1;
     0 0 0 0;
     0 0 0 0];

B = [0 0;
     0 0;
     1 0;
     0 1];

% [2] Controllability test
rankCO = rank(ctrb(A,B));
fprintf("----Controllabiltiy test for virtual control input?: ");
if (rankCO >= height(q))
    fprintf("YES----\n");
else
    fprintf("NO----\n");
end

% [3] Design State-feedback Control
% Let eigenvalues for state-feedback design be as follows
% lambda = [-7.1+i, -4.5, -7.1-i, -1.9];    % When I made a mistake with A matrix, wasted my time tuning this
% lambda = [-2, -5, -3, -4];                % Last asignment values too low to push the rrbot to upward config
lambda = [-13, -10, -12, -6];   
A = double(A);
B = double(B);
global K;                   % To access this in other scripts
% K diemension should be m*n.. Here, 2*4
K = place(A, B, lambda);
fprintf("-----State Feedback Control-----\n");
fprintf("lambda = \n");
disp(lambda);
fprintf("K = \n");
disp(K);










