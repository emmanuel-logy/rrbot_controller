clear all; close all; clc;

%% STEP 1 to 5: Running previous hw script to get EOM
rrbot_dyn


%% STEP 6: Finding equillibrium point
% Use the original second-order ODE instead of state space rep for this fn
% At equilibrium point, the vel and acc is zero
% Also, here we are trying to find eq point when there is no input
EOM_eq = subs(EOM, [q1d, q1dd, q2d, q2dd, u1, u2], [0,0,0,0,0,0]);
sol = solve(EOM_eq == 0, [q1, q2]);

% We get 3 equilibirum points and just printing the equilibrium points
sol_print = [sol.q1, sol.q2];
for itr = 1:height(sol_print)     % height(A) gives row
    fprintf(['-----Equilibirum Point ', num2str(itr), '-----\n', '[q1, q2] --> ']);
    disp(sol_print(itr,:));
end


%% STEP 7: Linearize our system to continue our analysis
A = jacobian(Xd, X);
B = jacobian(Xd, u);
% Substitute robot diemension and other constants from HW to A and B
A = subs(A, [g,m1,m2,I1,I2,l1,l2,r1,r2], [9.81,1,1,0.084,0.084,1,1,0.45,0.45]);
B = subs(B, [g,m1,m2,I1,I2,l1,l2,r1,r2], [9.81,1,1,0.084,0.084,1,1,0.45,0.45]);
fprintf("-----Linearizing our system gives the following A and B matrcies-----")
fprintf("\nA: \n");
disp(A);
fprintf("\nB: \n");
disp(B);


%% STEP 8: Stability Analysis at each equilibrium point
% Substitute equilibirum points to A
for itr = 1:height(sol_print)
    tmp_q1 = double(sol_print(itr,1));
    tmp_q2 = double(sol_print(itr,2));
    tmp_A = A;                     % Dont change original A with variables
    tmp_A = subs(tmp_A, [X(1),X(2),X(3),X(4)], [tmp_q1,tmp_q2,0,0]);
    tmp_A = double(tmp_A);  % to convert symbols to decimal values
    eigA = eig(tmp_A);
    
    fprintf(['-----Eigen Values at equilibirum point ', num2str(itr), '-----\n', '[q1, q2] --> ']);
    disp(sol_print(itr,:));
    
    fprintf('eigA = \n');
    disp(eigA);
    
    % check if all entries in eigA are -ve
    if ( min(round(real(eigA))<0) == 0 )
        fprintf("Asymptotically Stable?: NO\n");
    else
        fprintf("Asymptotically Stable?: YES\n");
    end
end


%% STEP 9: Controllability Analysis for upward configuration
% Upward Config is [q1, q2] --> [0, 0]
% Let's change the original A and q1, q2 from here as it is what we
% consider going further
q1 = 0;
q2 = 0;
A = subs(A, [X(1),X(2),X(3),X(4)], [q1,q2,0,0]);
A = double(A);  
B = subs(B, [X(1),X(2),X(3),X(4)], [q1,q2,0,0]);
B = double(B);
rankCO = rank(ctrb(A,B));
if (rankCO >= height(q))
    fprintf("----Controllable at upward config?: YES----\n");
else
    fprintf("----Controllable at upward config?: NO----\n");
end

% For reporting purpose
fprintf("-----Linearization at upward configuration gives the following A and B matrices-----")
fprintf("\nA: \n");
disp(A);
fprintf("\nB: \n");
disp(B);


%% STEP 10: Design State-feedback Control
% Let eigenvalues for state-feedback design be as follows
syms k1 k2 k3 k4 k5 k6 k7 k8 lambda
lambda = [-2, -5, -3, -4];
A = double(A);
B = double(B);
global K;                   % To access this in other scripts
K = place(A, B, lambda);
fprintf("-----State Feedback Control-----\n");
fprintf("lambda = \n");
disp(lambda);
fprintf("K = \n");
disp(K);






