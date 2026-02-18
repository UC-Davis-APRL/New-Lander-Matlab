%% make_reference_trajectory.m
% Builds a smooth reference trajectory xRef(:,k) for a rocket (NED).
% State: x = [pn pe pd u v w Phi Theta Psi p q r]'
% pd is DOWN, altitude = -pd.

clear; clc; close all;

%% ---------------- Settings ----------------
dt = 0.01;             % time step [s]
tf = 35.0;             % total time [s]
t  = 0:dt:tf;
N  = numel(t);

nx = 12;

% Nominal attitudes: level, yaw = 0
phi0 = 0; th0 = 0; psi0 = 0;

%% ---------------- Build a mission timeline ----------------
% We define segments with types:
%  - "move": smooth minimum-jerk from p0 -> p1 over [t0,t1]
%  - "hold": stay exactly at p over [t0,t1] with zero vel/acc

% Positions are in NED (pn, pe, pd). Remember: pd=-altitude.

% Example mission: climb to 50 m, hold, translate north, hold, descend.
segments = {};

% helper: convert altitude to pd
pd_from_alt = @(alt) -alt;

segments{end+1} = seg_move(0.0,  10.0, [0;0;pd_from_alt(0)],   [5;0;pd_from_alt(50)]);
segments{end+1} = seg_move(10.0,20.0,  [5;0;pd_from_alt(50)], [5;0;pd_from_alt(0)]);
segments{end+1} = seg_move(20.0,25.0,  [5;0;pd_from_alt(00)], [0;0;pd_from_alt(0)]);
segments{end+1} = seg_hold(25.0,35.0,  [0;0;pd_from_alt(0)])


% (Optional) safety: ensure segments cover [0,tf] without gaps/overlap
validate_segments(segments, tf);

%% ---------------- Generate reference position/velocity/acc ----------------
pRef   = zeros(3,N);    % [pn; pe; pd]
vRef   = zeros(3,N);    % inertial velocity reference (approx)
aRef   = zeros(3,N);    % inertial accel reference (approx)

for s = 1:numel(segments)
    seg = segments{s};
    idx = find(t >= seg.t0 & t <= seg.t1);

    if strcmp(seg.type, "hold")
        pRef(:,idx) = repmat(seg.p(:), 1, numel(idx));
        vRef(:,idx) = 0;
        aRef(:,idx) = 0;

    elseif strcmp(seg.type, "move")
        [p, v, a] = minjerk_segment(t(idx), seg.t0, seg.t1, seg.p0(:), seg.p1(:));
        pRef(:,idx) = p;
        vRef(:,idx) = v;
        aRef(:,idx) = a;
    else
        error("Unknown segment type");
    end
end

%% ---------------- Map to rocket state reference xRef ----------------
% Your rocket state uses body velocities (u,v,w) and Euler angles.
% For planning references, we can:
%  - set Euler angles to 0 (level) unless you want tilt planning
%  - approximate body velocities as inertial velocities (works if level/small angles)
%  - set angular rates to 0

xRef = zeros(nx,N);

xRef(1:3,:) = pRef;        % pn, pe, pd

% Approx: set u,v,w to inertial velocities north/east/down
% If you want, you can rotate inertial v into body frame using your rotation.
xRef(4,:) = vRef(1,:);     % u ~ pn_dot
xRef(5,:) = vRef(2,:);     % v ~ pe_dot
xRef(6,:) = vRef(3,:);     % w ~ pd_dot

xRef(7,:) = phi0;
xRef(8,:) = th0;
xRef(9,:) = psi0;

xRef(10:12,:) = 0;

%% ---------------- Build time-varying weights Qk ----------------
% Strong tracking on altitude during hold windows, lighter elsewhere.
Qbase = diag([ 5, 5, 30,  1,1,5,  50,50,5,  1,1,1 ]);
Qk = repmat(Qbase, 1,1,N);

% Identify hold intervals and boost altitude/vertical-rate weights
for s = 1:numel(segments)
    seg = segments{s};
    if strcmp(seg.type, "hold")
        idx = find(t >= seg.t0 & t <= seg.t1);
        for k = idx
            Qk(3,3,k) = Qk(3,3,k) + 3e4;  % pd
            Qk(6,6,k) = Qk(6,6,k) + 1e4;  % w ~ pd_dot
        end
    end
end

%% ---------------- Plot reference ----------------
altRef = -xRef(3,:);

figure; plot(t, altRef, 'LineWidth', 2); grid on;
xlabel('t [s]'); ylabel('Altitude [m]');
title('Reference Altitude');

figure; plot(t, xRef(1,:), 'LineWidth', 2); grid on;
xlabel('t [s]'); ylabel('pn [m]');
title('Reference pn');

figure; plot(t, xRef(6,:), 'LineWidth', 2); grid on;
xlabel('t [s]'); ylabel('w (approx pd\_dot) [m/s]');
title('Reference vertical rate');

disp("Reference generated: variables t, xRef, Qk in workspace.");

save('refTraj.mat','t','xRef','Qk','dt','aRef');

%% =======================================================================
%% Local functions
%% =======================================================================

function seg = seg_move(t0, t1, p0, p1)
seg = struct('type',"move", 't0',t0, 't1',t1, 'p0',p0, 'p1',p1);
end

function seg = seg_hold(t0, t1, p)
seg = struct('type',"hold", 't0',t0, 't1',t1, 'p',p);
end

function validate_segments(segments, tf)
% Basic checks: start at 0, end at tf, no gaps/overlaps (within tolerance)
tol = 1e-9;

t0s = zeros(numel(segments),1);
t1s = zeros(numel(segments),1);
for i = 1:numel(segments)
    t0s(i) = segments{i}.t0;
    t1s(i) = segments{i}.t1;
end

[~,ord] = sort(t0s);
t0s = t0s(ord); t1s = t1s(ord);

if abs(t0s(1) - 0.0) > tol
    warning("Segments do not start at t=0 (start=%.3f)", t0s(1));
end
if abs(t1s(end) - tf) > 1e-6
    warning("Segments do not end at tf (end=%.3f, tf=%.3f)", t1s(end), tf);
end

for i = 1:numel(segments)-1
    if t1s(i) < t0s(i) - tol
        error("Segment %d has t1 < t0", i);
    end
    if abs(t1s(i) - t0s(i+1)) > 1e-3
        warning("Gap/overlap between segment %d and %d: %.3f s", i, i+1, t0s(i+1)-t1s(i));
    end
end
end

function [p,v,a] = minjerk_segment(t, t0, t1, p0, p1)
% Minimum-jerk (quintic) interpolation for each axis independently:
% s(t) = 10τ^3 - 15τ^4 + 6τ^5, with τ=(t-t0)/(t1-t0)
% Ensures: position matches endpoints, and v=a=0 at endpoints.

T = (t1 - t0);
tau = (t - t0)/T;
tau = min(max(tau,0),1);

s  = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
sd = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / T;
sdd= (60*tau - 180*tau.^2 + 120*tau.^3) / (T^2);

dp = (p1 - p0);

p = p0 + dp.*s;
v = dp.*sd;
a = dp.*sdd;
end
