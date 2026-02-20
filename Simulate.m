import SensorFusion.*
import LQRDynamics.*
%This is the cascade control script,
%according to some papers and a sizeable amount of chatgpt cascade control
%is better as it m
clear; clc; close all;

% ---------------- %
%% Output Options %%
% ---------------- %

render = false
playSpeed = 5;

% -------------------- %
%% simulation options %%
% -------------------- %

% Rocket parameters and gravity
constants = [227,9.81,1,184,184,28];
% [m g L Ixx Iyy Izz]

%weights

%increase to penalize error
Qa = diag([80,80,5,40,40,1]);
%increase to prioritize using that input
Ra = diag([1,1,1]);

%increase to penalize error
Qp = diag([5,5,5,0.5,0.5,0.8]);
%increase to prioritize using that input
Rp  = diag([80,80,80]);

%alias for common constants
m=constants(1); g=constants(2);

% ------------------- %
%% actual simulation %%
% ------------------- % 

%trajectory import
load("ilqr_solution.mat","U","XA","t","dt","pRef_ned","vRef_ned","aRef_ned","vRef_body","aRef_body");

%LQR matrix finding functions
Kp=FindPositionGainMatrix([0,0,0,0,0,0],[0,0,0],constants,Qp,Rp);

KaMatrix = -FindTimeVariantAttitudeGainMatrix(XA,U,constants,Qa,Ra);

Ka = matrixtimesToSimulinkStruct(KaMatrix,t(1:end-1));

% --- sim aliase ---
ref.pos = vectortimesToSimulinkStruct(XA(1:3,:), t);
ref.vel = vectortimesToSimulinkStruct(XA(4:6,:), t);
ref.acl = vectortimesToSimulinkStruct(aRef_ned, t);
ref.rot = vectortimesToSimulinkStruct(XA(7:9,:), t);
ref.rat = vectortimesToSimulinkStruct(XA(10:12,:), t);


uff = vectortimesToSimulinkStruct(U,t(1:end-1));
initialCondition = XA(:,1); % use this one most of the time
% initialCondition = [0,0,0,0,0,0,0,0,0,0,0,0]'; % use this if you want to
% start with some state error from the trajectory

% --------------------------------- %
%% sensor fusion matrix generation %%
% --------------------------------- %

N = numel(t);
AA = zeros(12,12,N);
BB = zeros(12,4,N);

[A_fun, B_fun, ~] = makeDynamicsLinearizer();

for i = 1:N-1
    xk = XA(:,i);
    uk = U(:,i);

    xk = xk(:);  uk = uk(:);  constants = constants(:);
    assert(numel(xk)==12, "xk must be 12x1");
    assert(numel(uk)==4,  "uk must be 4x1");
    assert(numel(constants)==6, "constants must be 6x1");

    AA(:,:,i) = A_fun(xk, uk, constants);
    BB(:,:,i) = B_fun(xk, uk, constants);
end


% -----------%
%% simulate %%
% ---------- % 

% Run simulink model, comment out if you want to view an already
% simulated models visualization faster, simulink is often slow
mdl = "TVCmodel.slx";

results=sim(mdl);

% retrieve data from simulink model and format for rendering

%(Nx12) array
state=results.state(:,:);

%(Nx4) array
controls=results.controls(:,:);

%(Nx6) array
posError=-results.error(:,:);

% frame timesteps and number of frames from simulink model
times=results.tout';
frames = numel(state(:,1));


%divide state data for ease of access
rocketPos=state(:,1:3);
rocketRot=state(:,7:9);

%divide control data for ease of access
thrusts = controls(:,1);
alphaAngles = controls(:,2);
betaAngles = controls(:,3);

% --- calculate rocket direction vector from euler angles---

%define rocket direction as pointing in -z dir
rocketDir = zeros(frames,3);
rocketDir(:,3) = -1;

%rotate by euler angles in rocketRot using my nice little function
rocketDir = RotateEulerAnglesYPR(rocketDir,rocketRot(:,1),rocketRot(:,2),rocketRot(:,3));
% ----------------------------------------------------------

%define vector pointing towards target from rocket
errorDir = posError(:,1:3);



% rotate the vector pointing opposite the rocket by the thrust angles and
% the rotation of the rocket around Z to determine the direction of thrust,
% then scale to make it look reasonable
thrustDir = (thrusts/(m*g)) .* RotateEulerAnglesYPR(-rocketDir,alphaAngles,betaAngles,rocketRot(3));



if render

% ----------------- %
%% Animated figure %%
% ----------------- %

fig = figure(1);
clf(fig);

set(fig, 'Units', 'pixels', ...
         'Position', [100 100 960 540], ...   % <-- pick your video resolution
         'Resize', 'off');                    % <-- prevents frame size changes

ax = axes('Parent', fig);
set(ax, 'Units', 'normalized', ...
        'Position', [0.08 0.08 0.88 0.88]);   % fixed axes box inside figure

grid(ax, 'on');
hold(ax, 'off');



% Optional but recommended: lock axis behavior to avoid auto changes
axis(ax, 'manual');        % prevents auto-scaling/layout surprises
% axis(ax, 'vis3d');       % helps keep 3D camera stable (optional)

%% --- Quivers ---
rocketVec = quiver3(ax, NaN,NaN,NaN, NaN,NaN,NaN, 'r');
hold(ax, 'on');
engineVec = quiver3(ax, NaN,NaN,NaN, NaN,NaN,NaN, 'g');
errorVec  = quiver3(ax, NaN,NaN,NaN, NaN,NaN,NaN, 'b');

%% --- Video writer ---
writerObj = VideoWriter('test2','MPEG-4');
writerObj.FrameRate = 20; % optional; set if you want a specific fps
open(writerObj);

%% --- Loop ---
viewBox = [-5 5 -5 5 -5 5];   % constant camera box

for k = 1:10:frames-1

    % Update vectors
    rocketVec.XData = rocketPos(k,1); rocketVec.YData = rocketPos(k,2); rocketVec.ZData = rocketPos(k,3);
    rocketVec.UData = rocketDir(k,1); rocketVec.VData = rocketDir(k,2); rocketVec.WData = rocketDir(k,3);

    engineVec.XData = rocketPos(k,1); engineVec.YData = rocketPos(k,2); engineVec.ZData = rocketPos(k,3);
    engineVec.UData = thrustDir(k,1); engineVec.VData = thrustDir(k,2); engineVec.WData = thrustDir(k,3);

    errorVec.XData  = rocketPos(k,1); errorVec.YData  = rocketPos(k,2); errorVec.ZData  = rocketPos(k,3);
    errorVec.UData  = errorDir(k,1);  errorVec.VData  = errorDir(k,2);  errorVec.WData  = errorDir(k,3);

    % Move camera (axis limits) WITHOUT changing figure/axes size
    axis(ax, [ ...
        rocketPos(k,1)+viewBox(1), rocketPos(k,1)+viewBox(2), ...
        rocketPos(k,2)+viewBox(3), rocketPos(k,2)+viewBox(4), ...
        rocketPos(k,3)+viewBox(5), rocketPos(k,3)+viewBox(6)]);
    
    % Flip z axis because of reference frame
    set(ax, 'ZDir', 'reverse');
    drawnow limitrate;

    % Capture frame with fixed size
    F = getframe(fig);
    writeVideo(writerObj, F);

    % Real-time pacing (optional)
    %pause((times(k+1)-times(k))/playSpeed);
end

close(writerObj);
end

% ------------------ %
%% Graphs and Plots %%
% ------------------ %


figure; plot(t, -XA(3,:), 'b--', 'LineWidth',2); hold on; grid on;
plot(times, -rocketPos(:,3), 'LineWidth',2);
xlabel('t'); ylabel('Altitude [m]'); title('Altitude: reference vs achieved');
legend('planned','simulated');

figure; plot(t, XA(1,:), 'b--', 'LineWidth',2); hold on; grid on;
plot(times, rocketPos(:,1), 'LineWidth',2);
xlabel('t'); ylabel('Pn [m]'); title('North Pos: reference vs achieved');
legend('planned','simulated');

figure; plot(t, XA(2,:), 'b--', 'LineWidth',2); hold on; grid on;
plot(times, rocketPos(:,2), 'LineWidth',2);
xlabel('t'); ylabel('Pe [m]'); title('East Pos: reference vs achieved');
legend('planned','simulated');

% -------------------------------- %
%% local functions, no code below %%
% -------------------------------- %

function sig = matrixtimesToSimulinkStruct(M, t)
% Converts an A x B x T matrix into a Simulink time struct
%
% Inputs:
%   M : A x B x T numeric matrix
%   t : T x 1 or 1 x T time vector
%
% Output:
%   sig : struct usable by Simulink "From Workspace" block

arguments
    M (:,:,:) double
    t (:,1) double
end

[A,B,T] = size(M);

assert(numel(t) == T, 'Length of time vector must match 3rd dimension of M');

sig = struct();
sig.time = t(:);                    % T x 1
sig.signals.values = M;             % A x B x T
sig.signals.dimensions = [A B];     % required by Simulink
end

function ts = vectortimesToSimulinkStruct(data, t)
%AXTTOSIMULINKTIMESTRUCT Convert A×T array to Simulink time struct
%
%   ts = axtToSimulinkTimeStruct(data, t)
%
%   INPUTS:
%     data : A×T array (A signals, T time samples)
%     t    : T×1 or 1×T time vector
%
%   OUTPUT:
%     ts.time            -> T×1 time vector
%     ts.signals.values  -> T×A data matrix
%     ts.signals.dimensions -> A

    % Validate dimensions
    if size(data, 2) ~= numel(t)
        error('Number of columns in data must match length of time vector.');
    end

    % Ensure time is a column vector
    ts.time = t(:);

    % Transpose data to T×A for Simulink
    ts.signals.values = data.';

    % Number of signals
    ts.signals.dimensions = size(data, 1);
end

function ref = buildReferenceStruct(pRef_ned, vRef_ned, aRef_ned, t)
%BUILDREFERENCESTRUCT Create unified Simulink reference struct
%
% ref.pos, ref.vel, ref.acl are all Simulink-compatible

    ref.pos = vectortimesToSimulinkStruct(pRef_ned, t);
    ref.vel = vectortimesToSimulinkStruct(vRef_ned, t);
    ref.acl = vectortimesToSimulinkStruct(aRef_ned, t);

end