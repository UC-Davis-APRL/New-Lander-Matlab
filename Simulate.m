%This is the cascade control script,
%according to some papers and a sizeable amount of chatgpt cascade control
%is better as it m
%
%
%


%weights
Qa = diag([80,80,5,40,40,1]);
Ra = diag([1,1,1]);

Qp = diag([0.2,0.2,0.4,0.5,0.5,0.8]);
Rp  = diag([50,50,80]);

constants = [1,9.81,1,5,5,5]
%[m g L Ixx Iyy Izz]

%magical LQR matrix finding function
Ka=FindAttitudeGainMatrix([0,0,0,0,0,0],[0,0,0],constants,9.81,Qa,Ra)
Kp=FindPositionGainMatrix([0,0,0,0,0,0],[0,0,0],constants,Qp,Rp)

posTgt = [0,0,0,0,0,0];

initialCondition = [3,3,0,0,0,0,0,0,0,0,0,0];

% Run simulink model, comment out if you want to view a visualization
% faster, simulink is very slow
mdl = "TVCmodel.slx";
results=sim(mdl);

% retrieve data from simulink model
state=results.state(:,:);

controls=results.controls(:,:);
%

times=results.tout';
frames = numel(times);


%divide data for ease of access
pos=state(:,1:3)';
rot=state(:,7:9)';

rocketPos = zeros(3,1,frames);
rocketPos(1:3,1,:) = pos;

%calculate rocket direction vector from euler angles
rocketDir = zeros(3,1,frames);
rocketDir(3,1,:) = -1;
rocketDir = RotateEulerAnglesYPR(rocketDir,rot(1,:),rot(2,:),rot(3,:))


thrusts = zeros(1,1,frames);
thrusts(:,1,:) = controls(:,1)';

alphaAngles = controls(:,2);
betaAngles = controls(:,3);

% rotate the vector pointing opposite the rocket around roll axis, pitch
% axis, yaw axis, mabye its supposed to be the opposite, its x z y
thrustDir = (thrusts/20) .* RotateEulerAnglesYPR(-rocketDir,alphaAngles,betaAngles,rot(3));


%figure definitions

rocket = quiver3(NaN,NaN,NaN,NaN,NaN,NaN,'r');
hold on
engine = quiver3(NaN,NaN,NaN,NaN,NaN,NaN,'g');

% axis bounds for the graph
axis([-5,5,-5,5,-5,5]);

% flip z axis because of our choice of reference frame
set(gca, 'ZDir', 'reverse')
hold off


for t=1:frames-1 
    
    % the way this is animated is just taking each value of simulink data and rendering it with a pause

    %set figure data to respective values at timestep, might be faster to
    %remake without cell arrays
    rocket.XData = rocketPos(1,1,t); rocket.YData = rocketPos(2,1,t); rocket.ZData = rocketPos(3,1,t);
    rocket.UData = rocketDir(1,1,t); rocket.VData = rocketDir(2,1,t); rocket.WData = rocketDir(3,1,t);
    engine.XData = rocketPos(1,1,t); engine.YData = rocketPos(2,1,t); engine.ZData = rocketPos(3,1,t);
    engine.UData = thrustDir(1,1,t); engine.VData = thrustDir(2,1,t); engine.WData = thrustDir(3,1,t);
    
    
    drawnow
    
    %slows down to realtime? supposedly? 
    %frame times can be changed in the simulink model settings
    pause(times(t+1)-times(t))
end