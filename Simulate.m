%This is the cascade control script,
%according to some papers and a sizeable amount of chatgpt cascade control
%is better as it m

%weights

%increase to penalize error
Qa = diag([80,80,5,40,40,1]);
%increase to prioritize using that input
Ra = diag([1,1,1]);

%increase to penalize error
Qp = diag([2,2,2,0.5,0.5,0.8]);
%increase to prioritize using that input
Rp  = diag([80,80,80]);



% Rocket parameters and gravity
constants = [227,9.81,1,184,184,28];
% [m g L Ixx Iyy Izz]

%alias for common ones
m=constants(1); g=constants(2);

%literally just makes the frames play this much faster than "realtime"
%which isnt realtime anyways because its laggy
playSpeed = 4;


%magical LQR matrix finding function
Ka=FindAttitudeGainMatrix([0,0,0,0,0,0],[0,0,0],constants,9.81,Qa,Ra);
Kp=FindPositionGainMatrix([0,0,0,0,0,0],[0,0,0],constants,Qp,Rp);


% --- sim parameters ---
posTgt = [1,0,-50,0,0,0];

initialCondition = [0,0,0,0,0,0,0,0,0,0,0,0];
% ----------------------


% Run simulink model, comment out if you want to view an already
% simulated models visualization faster, simulink is often slow
mdl = "TVCmodel.slx";

results=sim(mdl);

% retrieve data from simulink model

%(Nx12) array
state=results.state(:,:);

%(Nx4) array
controls=results.controls(:,:);

% frame timesteps and number of frames from simulink model
times=results.tout';
frames = numel(times);


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
errorDir = posTgt(1:3)-rocketPos(:,1:3);



% rotate the vector pointing opposite the rocket by the thrust angles and
% the rotation of the rocket around Z to determine the direction of thrust,
% then scale to make it look reasonable
thrustDir = (thrusts/(m*g)) .* RotateEulerAnglesYPR(-rocketDir,alphaAngles,betaAngles,rocketRot(3));

%figure definitions
hold off
rocketVec = quiver3(NaN,NaN,NaN,NaN,NaN,NaN,'r');
hold on
engineVec = quiver3(NaN,NaN,NaN,NaN,NaN,NaN,'g');
errorVec = quiver3(NaN,NaN,NaN,NaN,NaN,NaN,'b');
% axis bounds for the graph


% flip z axis because of our fuckass choice of reference frame
set(gca, 'ZDir', 'reverse');



for t=1:frames-1 
    
    % the way this is animated is just taking each value of simulink data and rendering it with a pause
    %set figure data to respective values at timestep, might be faster to
    
    %assign xyz and uvw (tail position and vector) to coresponding values
    
   
    rocketVec.XData = rocketPos(t,1); rocketVec.YData = rocketPos(t,2); rocketVec.ZData = rocketPos(t,3);
    rocketVec.UData = rocketDir(t,1); rocketVec.VData = rocketDir(t,2); rocketVec.WData = rocketDir(t,3);
    
    engineVec.XData = rocketPos(t,1); engineVec.YData = rocketPos(t,2); engineVec.ZData = rocketPos(t,3);
    engineVec.UData = thrustDir(t,1); engineVec.VData = thrustDir(t,2); engineVec.WData = thrustDir(t,3);
    
    errorVec.XData = rocketPos(t,1); errorVec.YData = rocketPos(t,2); errorVec.ZData = rocketPos(t,3);
    errorVec.UData = errorDir(t,1); errorVec.VData = errorDir(t,2); errorVec.WData = errorDir(t,3);

    % make axis follow rocket
    viewBox = [-5,5,-5,5,-5,5];
    axis([rocketPos(t,1)+viewBox(1),... %-x
          rocketPos(t,1)+viewBox(2),... % x
          rocketPos(t,2)+viewBox(3),... %-y
          rocketPos(t,2)+viewBox(4),... % y
          rocketPos(t,3)+viewBox(5),... %-z
          rocketPos(t,3)+viewBox(6)]);  % z
    
    
    %RENDER!!!!!!! YIPPPEEEEE
    drawnow;
    
    %slows down to realtime? supposedly? 
    %frame times can be changed in the simrulink model settings
    pause((times(t+1)-times(t))/playSpeed);
end