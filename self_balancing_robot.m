%% MAE 506 Project: Self-Balancing Robot
% Linearized cart-pendulum model with full-state feedback.
% State vector: z = [x; x_dot; theta; theta_dot]
% Input: horizontal drive force u (N)
% SISO output: cart position y = x (m)
%
% Angle theta is measured from the upright position. The controller uses
% state feedback and assumes x, x_dot, theta, and theta_dot are available.

clear;
clc;
close all;

%% Physical parameters
M = 0.50;       % cart and wheelbase mass (kg)
m = 0.20;       % robot body mass (kg)
l = 0.30;       % pivot-to-center-of-mass distance (m)
I = 0.006;      % body moment of inertia about its center of mass (kg*m^2)
b = 0.10;       % viscous cart damping (N*s/m)
g = 9.81;       % gravitational acceleration (m/s^2)

J = I + m*l^2;
p = I*(M + m) + M*m*l^2;

%% Linearized state-space model about the upright position
% The nonlinear cart-pendulum equations are linearized at theta = 0 and
% theta_dot = 0. The theta^2 and theta_dot^2 terms are omitted.
A = [0, 1, 0, 0;
     0, -J*b/p, -m^2*g*l^2/p, 0;
     0, 0, 0, 1;
     0, m*l*b/p, (M + m)*m*g*l/p, 0];

B = [0;
     J/p;
     0;
     -m*l/p];

% The measured SISO output is cart position.
C = [1, 0, 0, 0];
D = 0;

%% System property analysis
controllabilityMatrix = ctrb(A, B);
observabilityMatrix = obsv(A, C);

controllabilityRank = rank(controllabilityMatrix);
observabilityRank = rank(observabilityMatrix);
numberOfStates = size(A, 1);

fprintf('Controllability rank: %d of %d\n', ...
    controllabilityRank, numberOfStates);
fprintf('Observability rank:   %d of %d\n', ...
    observabilityRank, numberOfStates);
fprintf('Open-loop eigenvalues:\n');
disp(eig(A));

%% State-feedback controller
% Pole placement moves the closed-loop poles into the left half-plane.
desiredPoles = [-2.0, -2.5, -3.0, -3.5];
K = place(A, B, desiredPoles);

Acl = A - B*K;

% Feedforward gain gives zero steady-state error for a constant cart
% position command in this linear model.
Nbar = -1/(C*(Acl\B));

fprintf('State-feedback gain K:\n');
disp(K);
fprintf('Reference feedforward gain Nbar: %.6f\n', Nbar);
fprintf('Closed-loop eigenvalues:\n');
disp(eig(Acl));

%% Simulate an initial tilt, an external push, and a position command
initialTilt = deg2rad(8);
zInitial = [0; 0; initialTilt; 0];

tOutput = (0:0.005:8)';
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', 0.005);

[t, z] = ode45(@(t, z) closedLoopDynamics(t, z, A, B, K, Nbar), ...
    tOutput, zInitial, options);

cartPosition = z(:, 1);
bodyAngleDegrees = rad2deg(z(:, 3));
reference = 0.10*(t >= 4.0);
pushForce = 1.0*((t >= 2.0) & (t <= 2.1));
controlForce = -z*K' + Nbar*reference;
totalForce = controlForce + pushForce;

%% Performance measures
referenceStepIndex = find(t >= 4.0, 1, 'first');
positionError = abs(cartPosition(referenceStepIndex:end) - 0.10);
suffixMaximumError = flipud(cummax(flipud(positionError)));
settlingIndexRelative = find(suffixMaximumError <= 0.002, 1, 'first');

if isempty(settlingIndexRelative)
    settlingTime = NaN;
else
    settlingIndex = referenceStepIndex + settlingIndexRelative - 1;
    settlingTime = t(settlingIndex) - 4.0;
end

finalPositionError = cartPosition(end) - 0.10;
finalAngleErrorDegrees = bodyAngleDegrees(end);

fprintf('Position settling time after 0.10 m command (2%% band): %.3f s\n', ...
    settlingTime);
fprintf('Final position error at %.1f s: %.6f m\n', t(end), finalPositionError);
fprintf('Final body angle at %.1f s: %.4f deg\n', t(end), finalAngleErrorDegrees);
fprintf('Maximum absolute body angle: %.3f deg\n', max(abs(bodyAngleDegrees)));
fprintf('Maximum absolute cart position: %.4f m\n', max(abs(cartPosition)));
fprintf('Maximum absolute applied force: %.3f N\n', max(abs(totalForce)));

%% Plot simulation results
figure('Color', 'w', 'Name', 'Self-Balancing Robot Results');

subplot(3, 1, 1);
plot(t, cartPosition, 'LineWidth', 1.6, 'Color', [0.60, 0.10, 0.20]);
hold on;
plot(t, reference, '--', 'LineWidth', 1.2, 'Color', [0.15, 0.15, 0.15]);
grid on;
ylabel('Position (m)');
title('Cart Position Tracking');
legend('Actual position', 'Reference', 'Location', 'best');

subplot(3, 1, 2);
plot(t, bodyAngleDegrees, 'LineWidth', 1.6, 'Color', [0.10, 0.40, 0.60]);
hold on;
yline(0, ':', 'Color', [0.15, 0.15, 0.15]);
grid on;
ylabel('Angle (deg)');
title('Robot Body Tilt');

subplot(3, 1, 3);
plot(t, totalForce, 'LineWidth', 1.6, 'Color', [0.20, 0.45, 0.25]);
hold on;
yline(0, ':', 'Color', [0.15, 0.15, 0.15]);
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Applied Drive Force');

sgtitle('Linearized Self-Balancing Robot with State Feedback');

%% Local function for the closed-loop simulation
function dz = closedLoopDynamics(t, z, A, B, K, Nbar)
    % Command the cart to move 0.10 m at t = 4 seconds.
    if t >= 4.0
        positionReference = 0.10;
    else
        positionReference = 0.0;
    end

    % Apply a brief 1 N push between 2.0 and 2.1 seconds.
    if t >= 2.0 && t <= 2.1
        disturbanceForce = 1.0;
    else
        disturbanceForce = 0.0;
    end

    controlForce = -K*z + Nbar*positionReference;
    dz = A*z + B*(controlForce + disturbanceForce);
end
