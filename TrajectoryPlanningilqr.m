function ilqr_track_refTraj()
%% iLQR that tracks a precomputed reference trajectory stored in refTraj.mat
% Uses augmented state xa = [x; u_prev], decision variable du = Delta u.
% State x = [pn pe pd u v w Phi Theta Psi p q r]' (12x1)
% Control u = [T Alph Bet Rcs]' (4x1)
%
% Required in refTraj.mat:
%   t     : 1xN or Nx1
%   xRef  : 12xN
%
% Optional in refTraj.mat:
%   Qk    : 12x12xN (time-varying state weights)
%   dt    : scalar (else inferred from t)

clear; clc; close all;

%% ---------------------- Load reference ----------------------
S = load('refTraj.mat');

assert(isfield(S,'t') && isfield(S,'xRef'), ...
    'refTraj.mat must contain variables t and xRef.');

t = S.t(:)';                    % force row
xRef = S.xRef;                  % 12xN expected

assert(size(xRef,1) == 12, 'xRef must be 12xN.');
N = size(xRef,2);

if isfield(S,'dt')
    dt = S.dt;
else
    assert(numel(t) == N, 't length must match xRef columns.');
    dt = mean(diff(t));
end

tf = t(end);

fprintf("Loaded refTraj.mat with N=%d, dt=%.6g, tf=%.3f s\n", N, dt, tf);

%% ---------------------- Planned velocity & acceleration in NED and BODY ----------------------
% Reference position in NED (pn, pe, pd)
pRef_ned = xRef(1:3,:);     % 3xN
tcol = t(:);                % Nx1 for gradient

% --- NED refs from position (kinematic) ---
vRef_ned = zeros(3,N);
aRef_ned = zeros(3,N);
for i = 1:3
    pi = pRef_ned(i,:).';          % Nx1
    vi = gradient(pi, tcol);       % dp/dt
    ai = gradient(vi, tcol);       % d2p/dt2
    vRef_ned(i,:) = vi.';          % 1xN
    aRef_ned(i,:) = ai.';          % 1xN
end

% Optional smoothing (recommended)
doSmooth = true;
if doSmooth
    wwin = max(5, 2*floor(0.15/dt)+1); % ~150 ms window, odd
    for i = 1:3
        vRef_ned(i,:) = smoothdata(vRef_ned(i,:), 'movmean', wwin);
        aRef_ned(i,:) = smoothdata(aRef_ned(i,:), 'movmean', wwin);
    end
end

% --- BODY refs using reference attitude ---
vRef_body = zeros(3,N);
aRef_body = zeros(3,N);

PhiRef   = xRef(7,:);
ThetaRef = xRef(8,:);
PsiRef   = xRef(9,:);

pRef = xRef(10,:);  % body rates
qRef = xRef(11,:);
rRef = xRef(12,:);

for k = 1:N
    % Rotation from BODY->NED from your own kinematics (same as used in xdot(1:3))
    cphi = cos(PhiRef(k));   sphi = sin(PhiRef(k));
    cth  = cos(ThetaRef(k)); sth  = sin(ThetaRef(k));
    cps  = cos(PsiRef(k));   sps  = sin(PsiRef(k));

    R_b2n = [ cth*cps,  sphi*sth*cps - cphi*sps,  cphi*sth*cps + sphi*sps;
              cth*sps,  sphi*sth*sps + cphi*cps,  cphi*sth*sps - sphi*cps;
             -sth,      sphi*cth,               cphi*cth ];

    R_n2b = R_b2n.'; % rotation matrices are orthonormal

    % body velocity reference
    v_b = R_n2b * vRef_ned(:,k);
    vRef_body(:,k) = v_b;

    % body "acceleration" consistent with vdot_body dynamics:
    % vdot_b = R_n2b * a_ned  -  omega x v_b
    omega_b = [pRef(k); qRef(k); rRef(k)];
    a_b = R_n2b * aRef_ned(:,k) - cross(omega_b, v_b);
    aRef_body(:,k) = a_b;
end

fprintf("Built refs: vRef_ned/aRef_ned and vRef_body/aRef_body (all 3xN). Smooth=%d\n", doSmooth);


%% ---------------------- Rocket constants ----------------------
% constants = [m g L Ixx Iyy Izz]
constants = [227,9.81,1,184,184,28];
m = constants(1); g = constants(2);

nx = 12;
nu = 4;
nxa = nx + nu;

%% ---------------------- Control bounds (on actual u) ----------------------
uMin = [0;          -deg2rad(12); -deg2rad(12); -8.0];
uMax = [3.0*m*g;     deg2rad(12);  deg2rad(12);  8.0];

%% ---------------------- Weights ----------------------
% If Qk is provided in .mat, use it. Otherwise create a default.
if isfield(S,'Qk')
    Qk = S.Qk;
    assert(all(size(Qk) == [12 12 N]), 'Qk must be 12x12xN.');
else
    Qbase = diag([ ...
        5, 5, 30, ...   % pn pe pd
        1, 1, 5,  ...   % u v w
        50,50,5, ...    % Phi Theta Psi
        1, 1, 1  ...    % p q r
    ]);
    Qk = repmat(Qbase,1,1,N);

    % Optional auto-boost during "holds" detected by small change in altitude ref
    altRef = -xRef(3,:);
    dAlt = [0, abs(diff(altRef))];
    holdIdx = dAlt < 1e-3; % "flat" segments
    for k = find(holdIdx)
        Qk(3,3,k) = Qk(3,3,k) + 2e4; % altitude tight during holds
        Qk(6,6,k) = Qk(6,6,k) + 8e3; % vertical rate damping during holds
    end
end

% Terminal weight (can be separate; here reuse last Qk slice, boosted)
Qf = Qk(:,:,end) * 10;

% Δu smoothing weights (Fix #3)
Rdu = diag([ ...
    2e-2, ...   % ΔT
    80.0, ...   % Δalpha
    80.0, ...   % Δbeta
    2.0  ...    % ΔRcs
]);

%% ---------------------- Initial conditions ----------------------
% Start at reference initial state (or you can set your own x0)
x0 = xRef(:,1);

uHover = [m*g; 0; 0; 0];
xa0 = [x0; uHover];

%% ---------------------- iLQR options ----------------------
opts.maxIter = 80;
opts.tolCost = 1e-6;

opts.lambdaInit   = 1e-3;
opts.lambdaFactor = 10;
opts.lambdaMax    = 1e8;

opts.lineSearchAlphas = [1.0 0.5 0.25 0.1 0.05 0.02 0.01];

% Finite-difference step sizes (for augmented state and du)
fd.epsX = 1e-6;
fd.epsU = 1e-6;

%% ---------------------- Initial guess (DU) ----------------------
DU = zeros(nu, N-1);  % keeping u near uHover at first

[XA, cost] = rollout_ref_aug(xa0, DU, dt, constants, uMin, uMax, xRef, Qk, Rdu, Qf);
fprintf("Initial cost: %.6g\n", cost);

%% ---------------------- Plot each iteration altitude ----------------------
figure(100); clf; hold on; grid on;
xlabel('Time [s]'); ylabel('Altitude [m]');
title('Altitude per iLQR iteration (tracking ref)');
plot(t, -xRef(3,:), 'k--', 'LineWidth', 2); % reference
plot(t, -XA(3,:), 'LineWidth', 1.2);
drawnow;

lambda = opts.lambdaInit;
costPrev = cost;

for iter = 1:opts.maxIter
    [A,B] = linearizeTrajectory_aug(XA, DU, dt, constants, fd);

    [lx, lu, lxx, luu, lux] = quadratizeCost_ref_aug(XA, DU, xRef, Qk, Rdu, Qf);

    [kff, Kfb, expectedDecrease, ok] = backwardPass(A,B,lx,lu,lxx,luu,lux,lambda);
    if ~ok
        lambda = min(lambda*opts.lambdaFactor, opts.lambdaMax);
        fprintf("Iter %d: backward pass failed, lambda -> %.3g\n", iter, lambda);
        if lambda >= opts.lambdaMax, break; end
        continue;
    end

    accepted = false;
    bestCost = inf; bestXA = []; bestDU = []; bestAlpha = NaN;

    for a = opts.lineSearchAlphas
        [XAnew, DUnew, cnew] = forwardPass_ref_aug( ...
            xa0, XA, DU, kff, Kfb, a, dt, constants, uMin, uMax, xRef, Qk, Rdu, Qf);

        if cnew < bestCost
            bestCost = cnew; bestXA = XAnew; bestDU = DUnew; bestAlpha = a;
        end
        if cnew < cost
            accepted = true;
            break;
        end
    end

    if accepted
        XA = bestXA; DU = bestDU; cost = bestCost;

        dCost = costPrev - cost;
        lambda = max(lambda/opts.lambdaFactor, 1e-12);

        fprintf("Iter %2d | cost %.6g | dCost %.3g | alpha %.3g | lambda %.3g | expDec %.3g\n", ...
            iter, cost, dCost, bestAlpha, lambda, expectedDecrease);

        % Plot altitude this iteration
        c = iter / opts.maxIter;
        plot(t, -XA(3,:), 'Color', [c 0 1-c], 'LineWidth', 1.0);
        drawnow;

        if abs(dCost) < opts.tolCost
            fprintf("Converged: small cost improvement.\n");
            break;
        end
        costPrev = cost;
    else
        lambda = min(lambda*opts.lambdaFactor, opts.lambdaMax);
        fprintf("Iter %2d | no improvement, lambda -> %.3g (best alpha %.3g cost %.6g)\n", ...
            iter, lambda, bestAlpha, bestCost);
        if lambda >= opts.lambdaMax, break; end
    end
end

%% ---------------------- Recover actual controls U ----------------------
U = zeros(nu, N-1);
for k = 1:N-1
    u_prev = XA(13:16,k);
    du = DU(:,k);
    [du, u] = clampDeltaU(u_prev, du, uMin, uMax);
    U(:,k) = u;
end
XA=XA(1:12,:)
save('ilqr_solution.mat', 'XA', 'U', 't', 'dt', ...
    'pRef_ned', 'vRef_ned', 'aRef_ned', ...
    'vRef_body', 'aRef_body');
%% ---------------------- Final plots ----------------------
figure; plot(t, -xRef(3,:), 'k--', 'LineWidth',2); hold on; grid on;
plot(t, -XA(3,:), 'LineWidth',2);
xlabel('t'); ylabel('Altitude [m]'); title('Altitude: reference vs achieved');
legend('ref','iLQR');

figure; plot(t, xRef(1,:), 'k--', 'LineWidth',2); hold on; grid on;
plot(t, XA(1,:), 'LineWidth',2);
xlabel('t'); ylabel('pN'); title('pn: reference vs achieved');
legend('ref','iLQR');

tu = t(1:end-1);
figure; plot(tu, U(1,:), 'LineWidth',1.6); grid on; xlabel('t'); ylabel('T [N]'); title('Thrust');
figure; plot(tu, rad2deg(U(2,:)), tu, rad2deg(U(3,:)), 'LineWidth',1.2);
grid on; xlabel('t'); ylabel('deg'); legend('\alpha','\beta'); title('Gimbal angles');
figure; plot(tu, U(4,:), 'LineWidth',1.6); grid on; xlabel('t'); ylabel('Rcs'); title('RCS');

disp("Done.");
end

%% =======================================================================
%% Rollout + linearize + cost (reference tracking, augmented Δu)
%% =======================================================================

function [XA, totalCost] = rollout_ref_aug(xa0, DU, dt, constants, uMin, uMax, xRef, Qk, Rdu, Qf)
nx = 12; nu = 4;
N  = size(DU,2) + 1;

XA = zeros(nx+nu, N);
XA(:,1) = xa0;

totalCost = 0;

for k = 1:N-1
    xa_k  = XA(:,k);
    xk    = xa_k(1:nx);
    u_prev = xa_k(nx+1:nx+nu);

    du = DU(:,k);
    [du, ~] = clampDeltaU(u_prev, du, uMin, uMax);

    XA(:,k+1) = rk4_discrete_aug(xa_k, du, dt, constants);

    dx = xk - xRef(:,k);
    totalCost = totalCost + dx'*Qk(:,:,k)*dx + du'*Rdu*du;
end

xN = XA(1:nx,end);
dN = xN - xRef(:,end);
totalCost = totalCost + dN'*Qf*dN;
end

function [A,B] = linearizeTrajectory_aug(XA, DU, dt, constants, fd)
nxa = size(XA,1);
nu  = size(DU,1);
N   = size(XA,2);

A = zeros(nxa,nxa,N-1);
B = zeros(nxa,nu ,N-1);

for k = 1:N-1
    xa = XA(:,k);
    du = DU(:,k);

    for i = 1:nxa
        d = zeros(nxa,1); d(i) = fd.epsX;
        fp = rk4_discrete_aug(xa + d, du, dt, constants);
        fm = rk4_discrete_aug(xa - d, du, dt, constants);
        A(:,i,k) = (fp - fm) / (2*fd.epsX);
    end

    for j = 1:nu
        d = zeros(nu,1); d(j) = fd.epsU;
        fp = rk4_discrete_aug(xa, du + d, dt, constants);
        fm = rk4_discrete_aug(xa, du - d, dt, constants);
        B(:,j,k) = (fp - fm) / (2*fd.epsU);
    end
end
end

function [lx, lu, lxx, luu, lux] = quadratizeCost_ref_aug(XA, DU, xRef, Qk, Rdu, Qf)
nx = 12;
nxa = size(XA,1);
nu  = size(DU,1);
N   = size(XA,2);

lx  = zeros(nxa, N);
lu  = zeros(nu,  N-1);
lxx = zeros(nxa, nxa, N);
luu = zeros(nu,  nu,  N-1);
lux = zeros(nu,  nxa, N-1);

for k = 1:N-1
    xk = XA(1:nx,k);
    dx = xk - xRef(:,k);
    du = DU(:,k);

    g = zeros(nxa,1);
    g(1:nx) = 2*Qk(:,:,k)*dx;
    lx(:,k) = g;

    lu(:,k) = 2*Rdu*du;

    lxx(:,:,k) = blkdiag(2*Qk(:,:,k), zeros(4));
    luu(:,:,k) = 2*Rdu;
    lux(:,:,k) = zeros(nu, nxa);
end

% terminal
xN  = XA(1:nx,N);
dxN = xN - xRef(:,N);

gN = zeros(nxa,1);
gN(1:nx) = 2*Qf*dxN;

lx(:,N) = gN;
lxx(:,:,N) = blkdiag(2*Qf, zeros(4));
end

function [XAnew, DUnew, costNew] = forwardPass_ref_aug(xa0, XA, DU, kff, Kfb, alpha, dt, constants, uMin, uMax, xRef, Qk, Rdu, Qf)
N = size(XA,2);

XAnew = zeros(size(XA));
DUnew = zeros(size(DU));
XAnew(:,1) = xa0;

for k = 1:N-1
    dxa = XAnew(:,k) - XA(:,k);

    du = DU(:,k) + alpha*kff(:,k) + Kfb(:,:,k)*dxa;

    u_prev = XAnew(13:16,k);
    [du, ~] = clampDeltaU(u_prev, du, uMin, uMax);

    DUnew(:,k) = du;
    XAnew(:,k+1) = rk4_discrete_aug(XAnew(:,k), du, dt, constants);
end

[~, costNew] = rollout_ref_aug(xa0, DUnew, dt, constants, uMin, uMax, xRef, Qk, Rdu, Qf);
end

%% =======================================================================
%% iLQR backward pass (LM regularization)
%% =======================================================================

function [kff, Kfb, dV, ok] = backwardPass(A,B,lx,lu,lxx,luu,lux,lambda)
[nx,~,K] = size(A);
[~,nu,~] = size(B);

kff = zeros(nu,K);
Kfb = zeros(nu,nx,K);

Vx  = lx(:,end);
Vxx = lxx(:,:,end);

dV = 0;
ok = true;
Iu = eye(nu);

for k = K:-1:1
    Ak = A(:,:,k);
    Bk = B(:,:,k);

    Qx  = lx(:,k)    + Ak'*Vx;
    Qu  = lu(:,k)    + Bk'*Vx;
    Qxx = lxx(:,:,k) + Ak'*Vxx*Ak;
    Quu = luu(:,:,k) + Bk'*Vxx*Bk;
    Qux = lux(:,:,k) + Bk'*Vxx*Ak;

    Quu_reg = Quu + lambda*Iu;

    [S,p] = chol(Quu_reg);
    if p ~= 0
        ok = false; return;
    end

    k_k = - (S \ (S' \ Qu));
    K_k = - (S \ (S' \ Qux));

    kff(:,k) = k_k;
    Kfb(:,:,k) = K_k;

    Vx  = Qx + K_k'*Quu*k_k + K_k'*Qu + Qux'*k_k;
    Vxx = Qxx + K_k'*Quu*K_k + K_k'*Qux + Qux'*K_k;
    Vxx = 0.5*(Vxx + Vxx');

    dV = dV + (-Qu'*k_k - 0.5*k_k'*Quu*k_k);
end
end

%% =======================================================================
%% Augmented RK4 + clamping utilities
%% =======================================================================

function xa_next = rk4_discrete_aug(xa, du, dt, constants)
nx = 12; nu = 4;
x = xa(1:nx);
u_prev = xa(nx+1:nx+nu);
u = u_prev + du;
Fw = zeros(3,1);
k1 = dynamics(x, u(1), u(2), u(3), u(4), constants,Fw);
k2 = dynamics(x + 0.5*dt*k1, u(1), u(2), u(3), u(4), constants,Fw);
k3 = dynamics(x + 0.5*dt*k2, u(1), u(2), u(3), u(4), constants,Fw);
k4 = dynamics(x + dt*k3,     u(1), u(2), u(3), u(4), constants,Fw);

x_next = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
xa_next = [x_next; u];   % u_prev_next = u
end

function [du_clamped, u_clamped] = clampDeltaU(u_prev, du, uMin, uMax)
u = u_prev + du;
u_clamped = min(max(u, uMin), uMax);
du_clamped = u_clamped - u_prev;
end