
%Gains
Q = diag([1,1,1,1,1,1,10,10,10,10,10,10]);
R = diag([1,500,500,10]);


%magical LQR matrix finding function
K=FindGainMatrix([0,0,0,0,0,0,0,0,0,0,0,0],[9.81,0,0,0],Q,R);

tgt = [0,0,0,0,0,0,0,0,0,0,0,0];

initialCondition = [-3,0,0,3,0,0,0,0,0,0,0,0];

% Run simulink model, comment out if you want to view a visualization
% faster, simulink is very slow
mdl = "TVCmodel.slx";
results=sim(mdl);

% retrieve data from simulink model
state=results.state(:,:);
controls=results.controls(:,:);
times=results.tout';
frames = numel(times);
%get rid of random useless leading singleton dimension

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
thrusterDir = (thrusts/20) .* RotateEulerAnglesYPR(-rocketDir,alphaAngles,betaAngles,rot(3));

% comment out if you want to reset one figure instead of making another one

%


cellRocketPos = num2cell(rocketPos());
cellRocketDir = num2cell(rocketDir());
cellThrusterDir = num2cell(thrusterDir());

rocket = quiver3(NaN,NaN,NaN,NaN,NaN,NaN,'r');
hold on
engine = quiver3(NaN,NaN,NaN,NaN,NaN,NaN,'g');

% axis bounds for the graph
axis([-5,5,-5,5,-5,5]);
set(gca, 'ZDir', 'reverse')
hold off


for t=1:frames-1 % the way this is animated is just taking each value of simulink data and rendering it with a pause

    [rocket.XData,rocket.YData,rocket.ZData] = cellRocketPos{:,:,t};
    [rocket.UData,rocket.VData,rocket.WData,] = cellRocketDir{:,:,t};
    [engine.XData,engine.YData,engine.ZData] = cellRocketPos{:,:,t};
    [engine.UData,engine.VData,engine.WData] = cellThrusterDir{:,:,t};
    
    
    drawnow
    
    %slows down to realtime? supposedly? its kinda laggy but the frame
    %times shoulnt be that long idk ig 9 fps is pretty slow
    pause(times(t+1)-times(t))
end