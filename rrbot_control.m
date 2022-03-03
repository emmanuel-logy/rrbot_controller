% clear all; close all; clc;

%% ROS Setup
rosshutdown;
rosinit;
j1_effort = rospublisher('/rrbot/joint1_effort_controller/command');
j2_effort = rospublisher('/rrbot/joint2_effort_controller/command');
JointStates = rossubscriber('/rrbot/joint_states');
tau1 = rosmessage(j1_effort);
tau2 = rosmessage(j2_effort);
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'rrbot';
req.UrdfParamName = 'robot_description';
req.JointNames = {'joint1','joint2'};
req.JointPositions = [deg2rad(30), deg2rad(45)];
resp = call(client,req,'Timeout',3);
tic;
t = 0;
X_Rec = [];
t_Rec = [];
u_Rec = [];

while(t < 10)
    t = toc;
    % read the joint states
    jointData = receive(JointStates);

    % Construct state vector
    X = [wrapTo2Pi(jointData.Position(1));
         wrapTo2Pi(jointData.Position(2));
         jointData.Velocity(1);
         jointData.Velocity(2)];

    % design your state feedback controller in the following
    global K;
    u = -K*X;
    tau1.Data = u(1);
    tau2.Data = u(2);
    send(j1_effort,tau1);
    send(j2_effort,tau2);

    % you can sample data here to be plotted at the end
    X_Rec = [X_Rec X];
    t_Rec = [t_Rec t];
    u_Rec = [u_Rec u];
end
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
% disconnect from roscore
rosshutdown;


%% Plotting the results
% Plot the q1, q1d, q2, q2d
figure;

subplot(3,2,1)
wrapTo2Pi(X_Rec(1,:));
plot(t_Rec, rad2deg(X_Rec(1,:)), 'b');
title('q1 vs t');
xlabel('t [sec]');
ylabel('q1 [deg]');
axis([0 10 -50 50]);  % Setting limits on the output
legend('q1');
grid on;

subplot(3,2,2)
wrapTo2Pi(X_Rec(2,:));
plot(t_Rec, rad2deg(X_Rec(2,:)), 'b');
title('q2 vs t');
xlabel('t [sec]');
ylabel('q2 [deg]');
axis([0 10 -50 70]);
legend('q2');
grid on;

subplot(3,2,3)
plot(t_Rec, rad2deg(X_Rec(3,:)), 'b');
title('q1d vs t');
xlabel('t [sec]');
ylabel('q1d [deg/sec]');
axis([0 10 -150 150]);
legend('q1d');
grid on;

subplot(3,2,4)
plot(t_Rec, rad2deg(X_Rec(4,:)), 'b');
title('q2d vs t');
xlabel('t [sec]');
ylabel('q2d [deg/sec]');
axis([0 10 -150 150]);
legend('q2d');
grid on;

subplot(3,2,5)
plot(t_Rec,u_Rec(1,:),'b');
title('u1 vs t');
xlabel('t [sec]');
ylabel('u1 [Nm]');
axis([0 10 -50 50]);
legend('u1');
grid on;

subplot(3,2,6)
plot(t_Rec,u_Rec(2,:),'r');
title('u2 vs t');
xlabel('t [sec]');
ylabel('u2 [Nm]');
axis([0 10 -50 50]);
legend('u2');
grid on;
