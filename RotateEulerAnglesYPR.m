function object = RotateEulerAnglesYPR(object,y,p,r)
%ROTATEEULERANGLESYPR rotates a vector
%   Detailed explanation goes here
arguments (Input)
    object double
    y double
    p double
    r double
end

Rx = zeros(3,3,numel(y));
c = cos(y);
s = sin(y);
Rx(1,1,:)=1;
Rx(2,2,:) = c;
Rx(2,3,:) = -s;
Rx(3,2,:) = s;
Rx(3,3,:) = c;

Ry = zeros(3,3,numel(p));
c = cos(p);
s = sin(p);
Ry(2,2,:)=1;
Ry(1,1,:) = c;
Ry(3,1,:) = -s;
Ry(1,3,:) = s;
Ry(3,3,:) = c;

Rz = zeros(3,3,numel(r));
c = cos(r);
s = sin(r);
Rz(3,3,:)=1;
Rz(1,1,:) = c;
Rz(2,1,:) = s;
Rz(1,2,:) = -s;
Rz(2,2,:) = c;

object = pagemtimes(Ry,pagemtimes(Rz,pagemtimes(Rx,object)));
end