
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This program is basical simulation 
% using sliding mode control for inverted pendulum
% 2018 / 10 / 12
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all
format short

Ts=1e-3;    % sampling time
l_bar = 2.0 % length of bar
M = 1.0     % [kg]
m = 0.3     % [kg]
g = 9.8     % [m/s^2]

% A matrix of state equation 
A = [0.0 1.0 0.0 0.0
     0.0 0.0 m * g / M, 0.0
     0.0 0.0 0.0 1.0
     0.0 0.0 g * (M + m) / (l_bar * M) 0.0]
% B matrix of state equation
B = [0.0
     1.0 / M
     0.0
     1.0 / (l_bar * M)]

% initial value of x
x0 = [0
      0
      0.3
      0]
 
% design of feedback gain with Linear quadratic method  
Q = diag([1e0 1e0 1e0 1e0])*1e1;
R = 1;
Klq = lqr(A,B,Q,R);

% design of hyper plane with Linear quadratic method
Q = diag([1e0 1e-1 1e0 1e0])*1e2;
R = 1;
S = lqr(A,B,Q,R);

% design of controller to achieve sliding mode 
% in final sliding mode method
Ksmcl  = inv(S*B)*S*A;  % linear state feedback gain
Ksmcnl = 4;             % nonlinear gain

% constant for smooth function
eta=1e-1




% simulation without deadzone
amp = 0; % value of deadzone in simulink

sim('designSmcLq')

figure
subplot(3,1,1)
plot(simTime,u(:))
hold on
grid on
plot(simTime,ulq(:))
title('u without deadzone')
subplot(3,1,2)
plot(simTime,X(:,1))
hold on
grid on
plot(simTime,Xlq(:,1))
title('x without deadzone')
subplot(3,1,3)
plot(simTime,X(:,3))
grid on
hold on
plot(simTime,Xlq(:,3))
title('Theta without deadzone')
xlabel('time')
legend('SMC', 'LQ')


% simulation without deadzone
amp = 1; % value of deadzone in simulink

sim('designSmcLq')

figure
subplot(3,1,1)
plot(simTime,u(:))
hold on
grid on
plot(simTime,ulq(:))
title('u with deadzone')
subplot(3,1,2)
plot(simTime,X(:,1))
hold on
grid on
plot(simTime,Xlq(:,1))
title('x with deadzone')
subplot(3,1,3)
plot(simTime,X(:,3))
grid on
hold on
plot(simTime,Xlq(:,3))
title('Theta with deadzone')
xlabel('time')
legend('SMC', 'LQ')
