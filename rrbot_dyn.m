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


%% STEP4: Euler-Lagrangian Equations of Motion
% EOM
DL_Dq = jacobian(L, q);  % D denotes partial diff and d is differentiation
DL_Dqd = jacobian(L, qd);
dDL_dtDqd = jacobian(DL_Dqd, [q; qd]) * [qd; qdd];          % Chain Rule
EOM = dDL_dtDqd - DL_Dq' - u;
EOM = simplify(EOM);
display(EOM(1));
display(EOM(2));


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
Xd = [X(3); X(4); sol.q1dd; sol.q2dd];
disp("-----State Space Representation-----");
display(Xd);

% For grouping of terms
% simplify(sol.q1dd)
% simplify(sol.q2dd)


%%
