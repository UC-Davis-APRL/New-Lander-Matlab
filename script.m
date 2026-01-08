

Q = diag([1,1,1,1,1,1,1,1,1,1,1,1]);
R = diag([100,1,1,1]);



K=FindGainMatrix([0,0,0,0,0,0,0,0,0,0,0,0],[9.81,0,0,0],Q,R);

tgt = [0,0,-10,1,0,0,0,0,0,0,0,0];

mdl = "TVCmodel.slx";
results=sim(mdl);

data=results.simout(1,1:12,:);
times=results.tout';
data=reshape(data,[12,length(times)])';
positions=data(:,1:3);
rotations=data(:,7:9);
dir = [sin(rotations(:,1)).*sin(rotations(:,2)),-cos(rotations(:,1)).*sin(rotations(:,2)),cos(rotations(:,2))];


figure;

for t=1:length(times)
    
    temppos = num2cell(positions(t,:));
    tempdir = num2cell(dir(t,:));
    
    quiver3(temppos{:},tempdir{:},'r')
    axis([-5,5,-5,5,-15,5])
    axis manual
    pause(0.1)
end