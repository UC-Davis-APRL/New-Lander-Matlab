
%Gains
Q = diag([1,1,1,1,1,1,10,10,10,10,10,10]);
R = diag([1,10,10,10]);


%magical LQR matrix finding function
K=FindGainMatrix([0,0,0,0,0,0,0,0,0,0,0,0],[9.81,0,0,0],Q,R);

tgt = [0,0,0,0,0,0,0,0,0,0,0,0];

initialCondition = [0,0,0,1,1,0,0,0,0,0,0,0]

% Run simulink model, comment out if you want to view a visualization
% faster, simulink is very slow
mdl = "TVCmodel.slx";
results=sim(mdl);

% retrieve data from simulink model
data=results.simout(:,:);
times=results.tout';

%get rid of random useless leading singleton dimension

%divide data for ease of access
pos=data(:,1:3);
rot=data(:,7:9);

%calculate rocket direction vector from euler angles
dir = [sin(rot(:,1)).*sin(rot(:,2)),-cos(rot(:,1)).*sin(rot(:,2)),cos(rot(:,2))];

% comment out if you want to reset one figure instead of making another one
figure;
%


for t=1:length(times) % the way this is animated is just taking each value of simulink data and rendering it with a pause
    
    temppos = num2cell(pos(t,:));
    tempdir = num2cell(dir(t,:));
    %mke a vector starting at the position and pointing in the direction of
    %the rocket
    quiver3(temppos{:},tempdir{:},'r')
    
    % comment out to remove trailf
    %
    %hold on
    %
    
    % axis bounds for the graph
    axis([-5,5,-5,5,-3,3])
    
    % 20 is the runtime set in the simulink file divide by number of frames
    % to get the frame time
    %pause(20/length(times))

    %i prefer this bc it runs faster, even if not close to realtime
    pause(0.1)
end