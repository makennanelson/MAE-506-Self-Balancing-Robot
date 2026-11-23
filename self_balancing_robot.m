%% MAE 506 Project: Self-Balancing Robot

clear; clc; close all;

% Model parameters
M = 0.50;
m = 0.20;
l = 0.30;
I = 0.006;
b = 0.10;
g = 9.81;
J = I + m*l^2;
p = I*(M + m) + M*m*l^2;

% State-space model
A = [0, 1, 0, 0;
     0, -J*b/p, -m^2*g*l^2/p, 0;
     0, 0, 0, 1;
     0, m*l*b/p, (M+m)*m*g*l/p, 0];

B = [0; J/p; 0; -m*l/p];
C = [1, 0, 0, 0];
D = 0;

% System properties
P = ctrb(A,B);       % Controllability matrix
Q = obsv(A,C);       % Observability matrix

fprintf('Controllability rank: %d of 4\n', rank(P));
fprintf('Observability rank:   %d of 4\n', rank(Q));
disp('Open-loop poles:');
disp(eig(A));

% State-feedback controller
desiredPoles = [-2, -2.5, -3, -3.5];
K = place(A,B,desiredPoles);
Acl = A - B*K;
Nbar = -1/(C*(Acl\B));   
disp('Controller gain K:');
disp(K);
fprintf('Reference gain Nbar: %.4f\n', Nbar);
disp('Closed-loop poles:');
disp(eig(Acl));

% Simulation
tspan = (0:0.005:8)';
initialState = [0; 0; deg2rad(8); 0];
[t,z] = ode45(@(t,z) robotDynamics(t,z,A,B,K,Nbar), tspan, initialState);
position = z(:,1);
angle = rad2deg(z(:,3));

% Position command starts at 4 seconds
reference = 0.10*(t >= 4);

% External push from 2.0 to 2.1 seconds
push = 1.0*(t >= 2.0 & t <= 2.1);

% Controller force and total force on the cart
control = -z*K' + Nbar*reference;
force = control + push;

% Performance measures
i = find(t >= 4, 1);
error = abs(position(i:end) - 0.10);

% Settling time
lastOutsideBand = find(error > 0.002, 1, 'last');

if isempty(lastOutsideBand)
    settlingTime = 0;
elseif lastOutsideBand == length(error)
    settlingTime = NaN;
else
    settlingTime = t(i + lastOutsideBand) - 4;
end

fprintf('Settling time: %.3f s\n', settlingTime);
fprintf('Final position error: %.6f m\n', position(end)-0.10);
fprintf('Final angle: %.4f deg\n', angle(end));
fprintf('Maximum angle: %.3f deg\n', max(abs(angle)));
fprintf('Maximum position: %.4f m\n', max(abs(position)));
fprintf('Maximum force: %.3f N\n', max(abs(force)));

% Plots
figure;
subplot(3,1,1)
plot(t,position,'b',t,reference,'k--');
grid on;
ylabel('Position (m)');
legend({'Position', 'Reference'});
title('Cart Position');
subplot(3,1,2)
plot(t,angle,'r');
grid on;
ylabel('Angle (deg)');
title('Robot Tilt');
subplot(3,1,3)
plot(t,force,'g');
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Drive Force');

% Animate the robot and save a video
L = 0.60;          
cartWidth = 0.35;
cartHeight = 0.15;
wheelRadius = 0.055;
cartY = 0.11;
pivotY = cartY + cartHeight;

figure;
hold on;
grid on;
axis equal;
xlim([min(position)-0.5, max(position)+0.5]);
ylim([-0.05, 1.05]);
xlabel('Position (m)');
ylabel('Height (m)');
title('Self-Balancing Robot Simulation');
plot(xlim, [0 0], 'k', 'LineWidth', 2);

% Create cart and wheels
cart = rectangle('Position', ...
    [position(1)-cartWidth/2, cartY, cartWidth, cartHeight], ...
    'FaceColor', [0.2 0.4 0.8]);
wheel1 = rectangle('Position', ...
    [position(1)-0.11-wheelRadius, 0, 2*wheelRadius, 2*wheelRadius], ...
    'Curvature', [1 1], 'FaceColor', [0.2 0.2 0.2]);
wheel2 = rectangle('Position', ...
    [position(1)+0.11-wheelRadius, 0, 2*wheelRadius, 2*wheelRadius], ...
    'Curvature', [1 1], 'FaceColor', [0.2 0.2 0.2]);

% Create pendulum
theta = z(1,3);
tipX = position(1) + L*sin(theta);
tipY = pivotY + L*cos(theta);
pendulum = plot([position(1), tipX], [pivotY, tipY], 'r-', 'LineWidth', 6);
topMarker = plot(tipX, tipY, 'o', 'MarkerSize', 16, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
timeLabel = text(min(position)-0.45, 0.95, '', 'FontSize', 12);
video = VideoWriter('robot_animation.mp4', 'MPEG-4');
video.FrameRate = 40;
open(video);

for k = 1:5:length(t)
    x = position(k);
    theta = z(k,3);
    tipX = x + L*sin(theta);
    tipY = pivotY + L*cos(theta);
    set(cart, 'Position', [x-cartWidth/2, cartY, cartWidth, cartHeight]);
    set(wheel1, 'Position', [x-0.11-wheelRadius, 0, 2*wheelRadius, 2*wheelRadius]);
    set(wheel2, 'Position', [x+0.11-wheelRadius, 0, 2*wheelRadius, 2*wheelRadius]);
    set(pendulum, 'XData', [x, tipX], 'YData', [pivotY, tipY]);
    set(topMarker, 'XData', tipX, 'YData', tipY);
    set(timeLabel, 'String', sprintf('Time = %.2f s', t(k)));
    drawnow;
    writeVideo(video, getframe(gcf));
end

close(video);

% Robot equations with feedback, command, and external push
function dz = robotDynamics(t,z,A,B,K,Nbar)
    if t >= 4
        reference = 0.10;
    else
        reference = 0;
    end
    if t >= 2.0 && t <= 2.1
        push = 1.0;
    else
        push = 0;
    end
    u = -K*z + Nbar*reference + push;
    dz = A*z + B*u;
end
