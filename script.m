
%Gains
Q = diag([1,1,1,1,1,1,10,10,10,10,10,10]);
R = diag([1,500,500,10]);


%magical LQR matrix finding function
K=FindGainMatrix([0,0,0,0,0,0,0,0,0,0,0,0],[9.81,0,0,0],Q,R);

tgt = [0,0,0,0,0,0,0,0,0,0,0,0];

initialCondition = [0,0,0,1,1,3,0,0,0,0,0,0];

% Run simulink model, comment out if you want to view a visualization
% faster, simulink is very slow
mdl = "TVCmodel.slx";
results=sim(mdl);

% retrieve data from simulink model
state=results.state(:,:);
controls=results.controls(:,:);
times=results.tout';
frames = numel(times)
%get rid of random useless leading singleton dimension

%divide data for ease of access
pos=state(:,1:3)';
rot=state(:,7:9)';

rocketPos = zeros(3,1,frames)
rocketPos(1:3,1,:) = pos

%calculate rocket direction vector from euler angles
rocketDir = zeros(3,1,frames)

rocketDir(1,1,:) = sin(rot(1,:)).*sin(rot(2,:))
rocketDir(2,1,:) = -cos(rot(1,:)).*sin(rot(2,:))
rocketDir(3,1,:) = cos(rot(2,:))


clear("pos","rot")
thrusts = zeros(1,1,frames)
thrusts(:,1,:) = controls(:,1)';
alphaAngles = controls(:,2);
betaAngles = controls(:,3);

Rx = zeros(3,3,frames);
c = cos(alphaAngles);
s = sin(alphaAngles);
Rx(1,1,:)=1;
Rx(2,2,:) = c;
Rx(2,3,:) = -s;
Rx(3,2,:) = s;
Rx(3,3,:) = c;

Ry = zeros(3,3,frames);
c = cos(betaAngles);
s = sin(betaAngles);
Ry(2,2,:)=1;
Ry(1,1,:) = c;
Ry(3,1,:) = -s;
Ry(1,3,:) = s;
Ry(3,3,:) = c;

% 3,1,N
thrusterDir = (thrusts/20) .* pagemtimes(Ry,pagemtimes(Rx,-rocketDir))

% comment out if you want to reset one figure instead of making another one
figure;
%


for t=1:frames % the way this is animated is just taking each value of simulink data and rendering it with a pause
    
    tempRocketPos = num2cell(rocketPos(:,:,t));
    tempRocketDir = num2cell(rocketDir(:,:,t));
    tempThrusterDir = num2cell(thrusterDir(:,:,t));
   
    %mke a vector starting at the position and pointing in the direction of
    %the rocket
    quiver3(tempRocketPos{:},tempRocketDir{:},'r')
    hold on
    quiver3(tempRocketPos{:},tempThrusterDir{:},'g')
    
    
    % axis bounds for the graph
    
    axis([-5,5,-5,5,-5,5]);
    hold off
    % 20 is the runtime set in the simulink file divide by number of frames
    % to get the frame time
    %pause(20/length(times))

    %i prefer this bc it runs faster, even if not close to realtime
    pause(0.1);
end