function [attitude,thrustCmd] = AccelToAttitude(desiredAccel, psiCmd,m,g)
arguments
    desiredAccel (3,1) double
    psiCmd (1,1) double
    m (1,1) double
    g (1,1) double
end
    %so i think this code needs to be changed if we want to add other
    %disturbance forces aside from gravity? im not sure though

    desiredForce = g*[0;0;1] - desiredAccel;

    forceMagnitude = norm(desiredForce);

    %to prevent div/0
    if forceMagnitude < 1e-9
        desiredForce = g*[0;0;1]
        forceMagnitude = norm(desiredForce)
    end
    
    thrustCmd = m*forceMagnitude;

    %unit vector pointing from back of rocket
    desiredThrustDir = desiredForce/forceMagnitude;
    
    %basically zero forever right now, supposedly having psiCmd go towards
    %where the rocket is pointing is a better implementation
    yawHeading = [cos(psiCmd); sin(psiCmd); 0];

    % math for making a rotation matrix that goes from world frame to a
    % frame with pos z towards the rear of the rocket, and with positive X
    % in the direction of psi
    c = cross(desiredThrustDir,yawHeading);

    if norm(c)<1e-6
        yawHeading = [cos(psiCmd+0.1); sin(psiCmd+0.1); 0];
        c = cross(desiredThrustDir,yawHeading);
    end
    %create frame pointing in desired thrust direction
    frameDir2 = c/norm(c);
    frameDir1 = cross(frameDir2,desiredThrustDir);
    frame = [frameDir1 frameDir2 desiredThrustDir]
    
    %turn that frame into desired euler angles
    thetaCmd = asin(-frame(3,1));
    phiCmd   = atan2(frame(3,2), frame(3,3));

    attitude = [phiCmd,thetaCmd,psiCmd];
