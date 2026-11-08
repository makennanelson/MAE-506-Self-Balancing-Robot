%% MAE 506 Project: Self-balancing robot, presentation animation
% Standalone. Designs the same state-feedback controller as
% self_balancing_robot.m (pole placement on the linearized model), then
% tests it on the full NONLINEAR robot equations and animates two robots:
%   left:  controller off -> the robot falls over
%   right: controller on  -> recovers from the starting tilt, survives a
%                            push, then drives forward to a new position
% Set saveVideo = true to export an MP4 for your slides.

clear; clc; close all;

%% Settings
tiltStart_deg = 8;      % starting tilt (deg)
pushForce     = 3;      % push on the base (N); your analysis script uses 1 N, 3 N is easier to see
pushTime      = 5;      % (s)
pushDuration  = 0.1;    % (s)
moveDistance  = 0.30;   % position command (m); your analysis script uses 0.10 m
moveTime      = 10;     % (s)
tEnd          = 16;     % (s)
saveVideo     = false;  % true -> export self_balancing_robot.mp4
playSpeed     = 1;      % 1 = real time, 0.5 = slow motion

%% Physical parameters (same as self_balancing_robot.m)
M = 0.50;       % cart and wheelbase mass (kg)
m = 0.20;       % robot body mass (kg)
l = 0.30;       % pivot-to-center-of-mass distance (m)
I = 0.006;      % body moment of inertia about its center of mass (kg*m^2)
b = 0.10;       % viscous cart damping (N*s/m)
g = 9.81;       % gravitational acceleration (m/s^2)
J = I + m*l^2;
p = I*(M + m) + M*m*l^2;

%% Controller design on the linearized model (same as your script)
A = [0, 1, 0, 0;
     0, -J*b/p, -m^2*g*l^2/p, 0;
     0, 0, 0, 1;
     0, m*l*b/p, (M + m)*m*g*l/p, 0];
B = [0; J/p; 0; -m*l/p];
C = [1, 0, 0, 0];
K = place(A, B, [-2.0, -2.5, -3.0, -3.5]);
Nbar = -1/(C*((A - B*K)\B));

%% Simulate the nonlinear robot, with and without the controller
P = struct('M',M,'m',m,'l',l,'J',J,'b',b,'g',g);
S = struct('tiltStart',deg2rad(tiltStart_deg),'pushForce',pushForce, ...
           'pushTime',pushTime,'pushDuration',pushDuration, ...
           'moveDistance',moveDistance,'moveTime',moveTime,'tEnd',tEnd);
dt = 0.001;
[t, zOff, ~, tFall] = simulateRobot(P, S, K, Nbar, false, dt);
[~, zOn, uOn]       = simulateRobot(P, S, K, Nbar, true,  dt);
ref = moveDistance*(t >= moveTime);

fprintf('Controller off: falls over at t = %.2f s\n', tFall);
fprintf('Controller on:  max tilt %.1f deg, max force %.2f N, final position %.3f m (target %.2f m)\n', ...
        max(abs(rad2deg(zOn(:,3)))), max(abs(uOn)), zOn(end,1), moveDistance);

%% Look and feel
thm.bg    = [0.05 0.06 0.09];
thm.panel = [0.09 0.10 0.14];
thm.floor = [0.08 0.09 0.12];
thm.txt   = [0.93 0.94 0.96];
thm.sub   = [0.62 0.65 0.72];
thm.dim   = [0.32 0.34 0.40];
thm.gold  = [1.00 0.80 0.30];
thm.green = [0.35 0.85 0.45];
thm.amber = [1.00 0.66 0.25];
thm.red   = [1.00 0.32 0.32];
thm.push  = [1.00 0.55 0.15];
thm.motor = [0.55 0.95 0.55];
thm.tire  = [0.14 0.15 0.19];
thm.metal = [0.62 0.64 0.70];
thm.visor = [0.06 0.07 0.10];
thm.eye   = [0.75 0.95 1.00];
c_off = [1.00 0.42 0.38];    % controller off: coral
c_on  = [0.20 0.90 0.70];    % controller on: teal

G.rw    = 0.055;   % wheel radius (drawn, m)
G.bodyW = 0.13;    % body width (drawn, m)
G.bodyL = 2*l;     % body length: center of mass at l above the axle

%% Build the figure
fig = figure('Color',thm.bg,'Position',[60 60 1280 720], ...
             'Name','Self-balancing robot','NumberTitle','off','InvertHardcopy','off');

xl = [-0.55 0.84];  yl = [-0.08 0.72];
axL = axes(fig,'Position',[0.02 0.40 0.47 0.48]);
axR = axes(fig,'Position',[0.51 0.40 0.47 0.48]);
setupScene(axL, thm, xl, yl);
setupScene(axR, thm, xl, yl);

% Target flag (right scene only)
hPole = plot(axR,[0 0],[0 0.16],'Color',thm.gold,'LineWidth',2);
hFlag = patch(axR,zeros(3,1),zeros(3,1),thm.gold,'EdgeColor','none');
hFlagTxt = text(axR,0,0.19,'target','Color',thm.gold,'FontSize',10,'HorizontalAlignment','center');

RL = makeRobot(axL, c_off, thm);
RR = makeRobot(axR, c_on,  thm);
motorArrow = makeArrow(axR, thm.motor, 3);
pushArrow  = makeArrow(axR, thm.push, 5);
hPushTxt   = text(axR,0,0,'PUSH','Color',thm.push,'FontSize',13,'FontWeight','bold', ...
                  'HorizontalAlignment','center','Visible','off');

HL = makeHud(fig, 0.02, c_off, thm, 'CONTROLLER OFF', 'no feedback');
HR = makeHud(fig, 0.51, c_on,  thm, 'CONTROLLER ON', 'state feedback (pole placement)');

annotation(fig,'textbox',[0.02 0.925 0.50 0.065],'String','SELF-BALANCING ROBOT', ...
           'Color',thm.txt,'FontSize',19,'FontWeight','bold','EdgeColor','none','VerticalAlignment','middle');
hTime = annotation(fig,'textbox',[0.80 0.925 0.18 0.065],'String','', ...
           'Color',thm.txt,'FontSize',16,'FontName','FixedWidth','EdgeColor','none', ...
           'HorizontalAlignment','right','VerticalAlignment','middle');
hEvent = annotation(fig,'textbox',[0.40 0.925 0.36 0.065],'String','', ...
           'Color',thm.gold,'FontSize',17,'FontWeight','bold','EdgeColor','none', ...
           'HorizontalAlignment','center','VerticalAlignment','middle');
annotation(fig,'textbox',[0.51 0.375 0.47 0.03], ...
           'String','Simulated on the full nonlinear equations  |  green arrow = motor force', ...
           'Color',thm.sub,'FontSize',9,'EdgeColor','none','HorizontalAlignment','right');

% Live plots
pd = 10;                                   % plot every 10th sample (0.01 s)
tp = t(1:pd:end);
thOffDeg = rad2deg(zOff(1:pd:end,3));
thOnDeg  = rad2deg(zOn(1:pd:end,3));
xOn      = zOn(1:pd:end,1);
refP     = ref(1:pd:end);

axA = axes(fig,'Position',[0.06 0.085 0.41 0.22]);
styleAxes(axA, thm);
tiltLim = max(12, tiltStart_deg + 4);
xlim(axA,[0 tEnd]);  ylim(axA,[-tiltLim tiltLim]);
plot(axA,[0 tEnd],[0 0],':','Color',thm.dim);
hPushLineA = plot(axA,[pushTime pushTime],[-tiltLim tiltLim],'--','Color',thm.push,'Visible','off');
hThOff = plot(axA,nan,nan,'Color',c_off,'LineWidth',2.2);
hThOn  = plot(axA,nan,nan,'Color',c_on,'LineWidth',2.2);
hThOnD = plot(axA,nan,nan,'o','MarkerSize',6,'MarkerFaceColor',c_on,'MarkerEdgeColor','w');
title(axA,'Body tilt (deg)','Color',thm.txt,'FontSize',12);
xlabel(axA,'Time (s)','Color',thm.sub);
legend(axA,[hThOff hThOn],{'Controller off','Controller on'}, ...
       'TextColor',thm.txt,'Color',thm.panel,'EdgeColor',thm.dim,'Location','southeast');

axB = axes(fig,'Position',[0.56 0.085 0.41 0.22]);
styleAxes(axB, thm);
xlim(axB,[0 tEnd]);
ylim(axB,[min([xOn; 0]) - 0.05, max([xOn; moveDistance]) + 0.08]);
hRef  = plot(axB,nan,nan,'--','Color',thm.gold,'LineWidth',1.6);
hX    = plot(axB,nan,nan,'Color',c_on,'LineWidth',2.2);
hXD   = plot(axB,nan,nan,'o','MarkerSize',6,'MarkerFaceColor',c_on,'MarkerEdgeColor','w');
title(axB,'Robot position (m)','Color',thm.txt,'FontSize',12);
xlabel(axB,'Time (s)','Color',thm.sub);
legend(axB,[hX hRef],{'Position','Target'}, ...
       'TextColor',thm.txt,'Color',thm.panel,'EdgeColor',thm.dim,'Location','northwest');

%% Animate
fps = 30;  dtF = 1/fps;
tf = 0:dtF*playSpeed:tEnd;

if saveVideo
    vid = VideoWriter('self_balancing_robot.mp4','MPEG-4');   % on Linux use 'Motion JPEG AVI'
    vid.FrameRate = fps;
    open(vid);
end

tStart = tic;
for n = 1:numel(tf)
    if ~ishandle(fig), break; end
    tn = tf(n);
    i  = min(numel(t), round(tn/dt) + 1);       % simulation sample for this frame
    ip = min(numel(tp), round(tn/(pd*dt)) + 1); % plot sample for this frame
    fallen = ~isnan(tFall) && tn >= tFall;

    % Robots
    updateRobot(RL, zOff(i,1), zOff(i,3), G);
    updateRobot(RR, zOn(i,1),  zOn(i,3),  G);
    if fallen
        set(RL.eyes,'Marker','x','MarkerEdgeColor',thm.red,'MarkerSize',9,'LineWidth',2);
    end

    % Target flag, motor force arrow, push arrow
    xr = ref(i);
    set(hPole,'XData',[xr xr]);
    set(hFlag,'XData',[xr; xr+0.06; xr],'YData',[0.16; 0.135; 0.11]);
    set(hFlagTxt,'Position',[xr 0.19 0]);
    setArrow(motorArrow, zOn(i,1) + sign(uOn(i))*G.rw, G.rw, 0.06*uOn(i), true);  % starts at the wheel edge
    pushOn = tn >= pushTime && tn <= pushTime + 0.4;
    setArrow(pushArrow, zOn(i,1) - G.rw - 0.22, G.rw, 0.21, pushOn);
    if pushOn
        set(hPushTxt,'Position',[zOn(i,1) - G.rw - 0.12, G.rw + 0.07, 0],'Visible','on');
    else
        set(hPushTxt,'Visible','off');
    end

    % HUDs
    setHudValues(HL, zOff(i,1), zOff(i,3), 0);
    setHudValues(HR, zOn(i,1),  zOn(i,3),  uOn(i));
    if fallen
        set(HL.status,'String','FELL OVER','BackgroundColor',thm.red);
    else
        set(HL.status,'String','TIPPING','BackgroundColor',thm.amber);
    end
    if abs(rad2deg(zOn(i,3))) > 1.5
        set(HR.status,'String','RECOVERING','BackgroundColor',thm.amber);
    elseif abs(zOn(i,1) - xr) > 0.01
        set(HR.status,'String','MOVING','BackgroundColor',thm.gold);
    else
        set(HR.status,'String','BALANCED','BackgroundColor',thm.green);
    end

    % Event banner
    if tn < 2.5
        set(hEvent,'String',sprintf('RELEASED AT %g^\\circ TILT', tiltStart_deg));
    elseif tn >= pushTime && tn < pushTime + 2
        set(hEvent,'String','PUSH!');
    elseif tn >= moveTime && tn < moveTime + 2.5
        set(hEvent,'String',sprintf('GO %g cm FORWARD', round(100*moveDistance)));
    else
        set(hEvent,'String','');
    end
    set(hTime,'String',sprintf('t = %5.2f s', tn));

    % Live plots
    set(hThOff,'XData',tp(1:ip),'YData',thOffDeg(1:ip));
    set(hThOn,'XData',tp(1:ip),'YData',thOnDeg(1:ip));
    set(hThOnD,'XData',tp(ip),'YData',thOnDeg(ip));
    set(hRef,'XData',tp(1:ip),'YData',refP(1:ip));
    set(hX,'XData',tp(1:ip),'YData',xOn(1:ip));
    set(hXD,'XData',tp(ip),'YData',xOn(ip));
    if tn >= pushTime
        set(hPushLineA,'Visible','on');
    end

    drawnow;
    if saveVideo
        writeVideo(vid, getframe(fig));
    else
        pause(max(0, n*dtF - toc(tStart)));
    end
end

if saveVideo
    lastFrame = getframe(fig);
    for k = 1:round(1.5*fps)                 % hold the final frame for 1.5 s
        writeVideo(vid, lastFrame);
    end
    close(vid);
    disp('Saved self_balancing_robot.mp4');
end

%% ---------------- Local functions ----------------
function [t, Z, U, tFall] = simulateRobot(P, S, K, Nbar, controlOn, dt)
    % Fixed-step RK4 on the nonlinear equations. The uncontrolled robot
    % stops when its body hits the ground.
    t = (0:dt:S.tEnd)';
    N = numel(t);
    Z = zeros(N,4);  U = zeros(N,1);
    z = [0; 0; S.tiltStart; 0];
    fallen = false;  tFall = NaN;
    thFloor = deg2rad(88);              % body lying on the ground
    for i = 1:N
        Z(i,:) = z';
        r = S.moveDistance*(t(i) >= S.moveTime);
        if controlOn
            u = -K*z + Nbar*r;
        else
            u = 0;
        end
        U(i) = u;
        if fallen, continue; end
        d = S.pushForce*(t(i) >= S.pushTime && t(i) <= S.pushTime + S.pushDuration);
        F = u + d;
        k1 = robotDynamics(z, F, P);
        k2 = robotDynamics(z + dt/2*k1, F, P);
        k3 = robotDynamics(z + dt/2*k2, F, P);
        k4 = robotDynamics(z + dt*k3, F, P);
        z = z + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        if abs(z(3)) >= thFloor
            z(3) = sign(z(3))*thFloor;  z(2) = 0;  z(4) = 0;
            fallen = true;  tFall = t(i);
        end
    end
end

function dz = robotDynamics(z, F, P)
    % Full nonlinear cart-pendulum equations. theta is measured from upright,
    % positive = leaning toward +x (same convention as the linear model).
    th = z(3);  thd = z(4);
    Mmat = [P.M + P.m,         P.m*P.l*cos(th);
            P.m*P.l*cos(th),   P.J];
    rhs  = [F - P.b*z(2) + P.m*P.l*sin(th)*thd^2;
            P.m*P.g*P.l*sin(th)];
    acc = Mmat \ rhs;
    dz = [z(2); acc(1); z(4); acc(2)];
end

function setupScene(ax, thm, xl, yl)
    hold(ax,'on');
    axis(ax,'equal');  axis(ax,[xl yl]);  axis(ax,'off');
    patch(ax,[xl(1); xl(2); xl(2); xl(1)],[yl(1); yl(1); 0; 0],thm.floor,'EdgeColor','none');
    plot(ax,xl,[0 0],'Color',thm.sub,'LineWidth',1.5);
    for xt = -0.5:0.1:0.8                         % floor ticks every 10 cm
        if abs(xt - round(xt*2)/2) < 1e-6
            plot(ax,[xt xt],[0 -0.03],'Color',thm.sub,'LineWidth',1.2);
            text(ax,xt,-0.055,sprintf('%.1f m',xt),'Color',thm.sub,'FontSize',9, ...
                 'HorizontalAlignment','center');
        else
            plot(ax,[xt xt],[0 -0.015],'Color',thm.dim);
        end
    end
end

function R = makeRobot(ax, c, thm)
    R.body   = patch(ax,zeros(6,1),zeros(6,1),c,'EdgeColor',0.6*c,'LineWidth',1.2);
    R.visor  = patch(ax,zeros(4,1),zeros(4,1),thm.visor,'EdgeColor','none');
    R.eyes   = plot(ax,nan,nan,'o','LineStyle','none','MarkerSize',8, ...
                    'MarkerFaceColor',thm.eye,'MarkerEdgeColor','none');
    R.wheel  = patch(ax,zeros(40,1),zeros(40,1),thm.tire,'EdgeColor',c,'LineWidth',2);
    R.spokes = plot(ax,nan,nan,'-','Color',thm.sub,'LineWidth',1.5);
    R.hub    = plot(ax,nan,nan,'o','MarkerSize',5,'MarkerFaceColor',thm.metal,'MarkerEdgeColor','none');
end

function updateRobot(R, x, theta, G)
    x0 = x;  y0 = G.rw;                          % axle position
    ev = [sin(theta), cos(theta)];               % along the body (up)
    eu = [cos(theta), -sin(theta)];              % across the body
    w = G.bodyW/2;  L = G.bodyL;  ch = 0.025;
    [X,Y] = bodyToWorld([-w, w, w, w-ch, -w+ch, -w], [0, 0, L-ch, L, L, L-ch], x0, y0, eu, ev);
    set(R.body,'XData',X(:),'YData',Y(:));
    [X,Y] = bodyToWorld([-0.045, 0.045, 0.045, -0.045], L - [0.14, 0.14, 0.05, 0.05], x0, y0, eu, ev);
    set(R.visor,'XData',X(:),'YData',Y(:));
    [X,Y] = bodyToWorld([-0.02, 0.02], (L - 0.095)*[1, 1], x0, y0, eu, ev);
    set(R.eyes,'XData',X,'YData',Y);
    a = linspace(0,2*pi,40)';
    set(R.wheel,'XData',x0 + G.rw*cos(a),'YData',y0 + G.rw*sin(a));
    phi = -x/G.rw + [0, pi/3, 2*pi/3];            % wheel rolls with the robot
    sx = [x0 + 0.85*G.rw*cos(phi); x0 - 0.85*G.rw*cos(phi); nan(1,3)];
    sy = [y0 + 0.85*G.rw*sin(phi); y0 - 0.85*G.rw*sin(phi); nan(1,3)];
    set(R.spokes,'XData',sx(:),'YData',sy(:));
    set(R.hub,'XData',x0,'YData',y0);
end

function [X,Y] = bodyToWorld(u, v, x0, y0, eu, ev)
    X = x0 + u*eu(1) + v*ev(1);
    Y = y0 + u*eu(2) + v*ev(2);
end

function A = makeArrow(ax, c, lw)
    A.line = plot(ax,[0 0],[0 0],'-','Color',c,'LineWidth',lw,'Visible','off');
    A.head = patch(ax,zeros(3,1),zeros(3,1),c,'EdgeColor','none','Visible','off');
end

function setArrow(A, x0, y0, dx, show)
    if ~show || abs(dx) < 0.005
        set(A.line,'Visible','off');  set(A.head,'Visible','off');
        return;
    end
    s = sign(dx);  hl = min(0.035, abs(dx));  hw = 0.02;
    xt = x0 + dx;
    set(A.line,'XData',[x0, xt - s*hl],'YData',[y0, y0],'Visible','on');
    set(A.head,'XData',[xt; xt - s*hl; xt - s*hl],'YData',[y0; y0 + hw; y0 - hw],'Visible','on');
end

function styleAxes(ax, thm)
    hold(ax,'on');  box(ax,'on');  grid(ax,'on');
    set(ax,'Color',thm.panel,'XColor',thm.sub,'YColor',thm.sub,'GridColor',[1 1 1], ...
           'GridAlpha',0.12,'FontSize',10);
end

function H = makeHud(fig, px, c, thm, name, sub)
    H.name = annotation(fig,'textbox',[px+0.01 0.835 0.30 0.05],'String',name, ...
        'Color',c,'FontSize',18,'FontWeight','bold','EdgeColor','none','VerticalAlignment','middle');
    H.sub = annotation(fig,'textbox',[px+0.01 0.795 0.35 0.04],'String',sub, ...
        'Color',thm.sub,'FontSize',11,'EdgeColor','none','VerticalAlignment','middle');
    H.val = annotation(fig,'textbox',[px+0.01 0.755 0.40 0.04],'String','', ...
        'Color',thm.txt,'FontSize',12,'FontName','FixedWidth','EdgeColor','none','VerticalAlignment','middle');
    H.status = annotation(fig,'textbox',[px+0.33 0.84 0.13 0.045],'String','', ...
        'Color',thm.bg,'BackgroundColor',thm.amber,'FontSize',12,'FontWeight','bold', ...
        'EdgeColor','none','HorizontalAlignment','center','VerticalAlignment','middle','FitBoxToText','off');
end

function setHudValues(H, x, theta, u)
    % Round to the displayed precision first (+0 turns -0 into 0)
    thd = round(rad2deg(theta)*10)/10 + 0;
    x   = round(x*1000)/1000 + 0;
    u   = round(u*100)/100 + 0;
    set(H.val,'String',sprintf('tilt %+5.1f^\\circ   x %+.3f m   force %+.2f N', thd, x, u));
end
