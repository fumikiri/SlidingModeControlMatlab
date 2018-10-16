
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
Q = diag([1e1 1e-1 1e0 1e0])*1e2;
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
title('horizontal position without deadzone')
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
title('horizontal position with deadzone')
subplot(3,1,3)
plot(simTime,X(:,3))
grid on
hold on
plot(simTime,Xlq(:,3))
title('Theta with deadzone')
xlabel('time')
legend('SMC', 'LQ')



X1lqd = decimate(Xlq(:,1), 100)
X3lqd = decimate(Xlq(:,3), 100)

X1d = decimate(X(:,1), 100)
X3d = decimate(X(:,3), 100)


h = figure('Position',[100 100 800 400]);
axis tight manual % this ensures that getframe() returns a consistent size
filename = 'testAnimated.gif';
for k = 1:(length(simTime)-1)/100+1
    subplot(1,2,1)
    yy = cos(X3lqd(k));
    yx = sin(X3lqd(k));
    x(1) = X1lqd(k)-1;
    x(2) = X1lqd(k)+1;
    plot(x,[0 0],'g-','LineWidth',5.5)
    ylim([-0.1 1.1])
    xlim([-3 3])
    hold on
    grid on
    plot([X1lqd(k) X1lqd(k)+yx],[0 yy],'b-','LineWidth',8.0)
    title({'Linear quadratic control' 'pos:' num2str(X1lqd(k),2) ' angle: ' num2str(X3lqd(k),2)})
    hold off
    drawnow
    subplot(1,2,2)
    yy = cos(X3d(k));
    yx = sin(X3d(k));
    x(1) = X1d(k)-1;
    x(2) = X1d(k)+1;
    plot(x,[0 0],'g-','LineWidth',5.5)
    ylim([-0.1 1.1])
    xlim([-3 3])
    hold on
    grid on
    plot([X1d(k) X1d(k)+yx],[0 yy],'b-','LineWidth',8.0)
    title({'Sliding mode control' 'pos: ' num2str(X1d(k),2)  '  angle: ' num2str(X3d(k),2) })
    hold off
    drawnow
      % Capture the plot as an image 
      frame = getframe(h); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,512); 
      % Write to the GIF File 
      if k == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',Ts*100); 
      else 
          imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',Ts*100); 
      end 
end
