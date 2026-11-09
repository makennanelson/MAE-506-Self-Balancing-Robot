%% MAE 506 Project: Self-Balancing Robot
% State: z = [position; velocity; angle; angular velocity]
% Input: motor force
% Output: cart position

clear; clc; close all;

%% Model parameters
M = 0.50;
m = 0.20;
l = 0.30;
I = 0.006;
b = 0.10;
g = 9.81;

J = I + m*l^2;
p = I*(M + m) + M*m*l^2;

%% State-space model: zdot = A*z + B*u, y = C*z
A = [0, 1, 0, 0;
     0, -J*b/p, -m^2*g*l^2/p, 0;
     0, 0, 0, 1;
     0, m*l*b/p, (M+m)*m*g*l/p, 0];

B = [0; J/p; 0; -m*l/p];
C = [1, 0, 0, 0];
D = 0;

%% System properties
P = ctrb(A,B);       % Controllability matrix
Q = obsv(A,C);       % Observability matrix

fprintf('Controllability rank: %d of 4\n', rank(P));
fprintf('Observability rank:   %d of 4\n', rank(Q));
disp('Open-loop poles:');
disp(eig(A));

%% State-feedback controller
desiredPoles = [-2, -2.5, -3, -3.5];
K = place(A,B,desiredPoles);

Acl = A - B*K;
Nbar = -1/(C*(Acl\B));   % Reference gain

disp('Controller gain K:');
disp(K);
fprintf('Reference gain Nbar: %.4f\n', Nbar);
disp('Closed-loop poles:');
disp(eig(Acl));

%% Simulation
tspan = (0:0.005:8)';
initialState = [0; 0; deg2rad(8); 0];

[t,z] = ode45(@(t,z) robotDynamics(t,z,A,B,K,Nbar), ...
              tspan, initialState);

position = z(:,1);
angle = rad2deg(z(:,3));

% Position command starts at 4 seconds
reference = 0.10*(t >= 4);

% External push from 2.0 to 2.1 seconds
push = 1.0*(t >= 2.0 & t <= 2.1);

% Controller force and total force on the cart
control = -z*K' + Nbar*reference;
force = control + push;

%% Performance measures
i = find(t >= 4, 1);
error = abs(position(i:end) - 0.10);

% Settling time: time after the command until error stays within 2%
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

%% Plots
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

%% Robot equations with feedback, command, and external push
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
