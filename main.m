% clear all; close all; clc;

%% Solving the State Space Equations
T = 10;                                     % Simulating for 10 seconds
y0 = [deg2rad(30),deg2rad(45),0,0];         % Initial conditions

[t,y] = ode45(@rrbot_ode, [0,T], y0);

%% Reconstruct control inputs
global K;
u = -K*y';


%% Plotting the results
figure;

subplot(3,2,1)
wrapTo2Pi(y(:,1));
plot(t,rad2deg(y(:,1)),'b');
title('q1 vs t');
xlabel('t [sec]');
ylabel('q1 [deg]');
axis([0 10 -50 50]);  % Setting limits on the output
legend('q1');
grid on;

subplot(3,2,2)
wrapTo2Pi(y(:,2));
plot(t,rad2deg(y(:,2)),'r');
title('q2 vs t');
xlabel('t [sec]');
ylabel('q2 [deg]');
axis([0 10 -50 50]);
legend('q2');
grid on;

subplot(3,2,3)
plot(t,rad2deg(y(:,3)),'b');
title('q1d vs t');
xlabel('t [sec]');
ylabel('q1d [deg/sec]');
axis([0 10 -150 150]);
legend('q1d');
grid on;

subplot(3,2,4)
plot(t,rad2deg(y(:,4)),'r');
title('q2d vs t');
xlabel('t [sec]');
ylabel('q2d [deg/sec]');
axis([0 10 -150 150]);
legend('q2d');
grid on;

subplot(3,2,5)
plot(t,u(1,:),'b');
title('u1 vs t');
xlabel('t [sec]');
ylabel('u1 [Nm]');
axis([0 10 -50 50]);
legend('u1');
grid on;

subplot(3,2,6)
plot(t,u(2,:),'r');
title('u2 vs t');
xlabel('t [sec]');
ylabel('u2 [Nm]');
axis([0 10 -50 50]);
legend('u2');
grid on;
%%