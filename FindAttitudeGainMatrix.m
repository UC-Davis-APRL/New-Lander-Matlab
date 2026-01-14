function Ka = FindAttitudeGainMatrix(xEq,uEq,TEq,Q,R)
arguments
    xEq 
    uEq
    TEq
    % Q matrix for state vector prioritization
    Q (6,6) double = diag([1,1,1,10,10,10]);
    % R matrix for input vector prioritization
    R (3,3) double = diag([1,1,1]);
    % Q matrix for state vector prioritizatio

end
%Ka=FindAttitudeGainMatrix([0,0,0,0,0,0],[0,0,0],9.81)

%constants
const = [1,9.81,1,5,5,5,TEq]
syms m g L Ixx Iyy Izz T
cSyms = [m g L Ixx Iyy Izz T]

% x
syms Phi Theta Psi p q r
xSyms = [Phi Theta Psi p q r]



% u1
syms Alph Bet Rcs
uSyms = [Alph Bet Rcs]

%angle
eq7 = p + sin(Phi)*tan(Theta)*q + cos(Phi)*tan(Theta)*r;
eq8 = cos(Phi)*q - sin(Phi)*r;
eq9 = (sin(Phi)/cos(Theta))*q + (cos(Phi)/cos(Theta))*r;
eq10 = (Iyy - Izz)*q*r/Ixx + (L*T*sin(Alph)*cos(Bet))/Ixx;
eq11 = (Izz - Ixx)*p*r/Iyy - (L*T*sin(Bet)*cos(Alph))/Iyy;
eq12 = (Ixx - Iyy)*p*q/Izz + Rcs/Izz;

Function = [eq7; eq8; eq9; eq10; eq11; eq12;];

% jacobians, find symbolic version and then substitute values.
jacX = jacobian(Function,[Phi Theta Psi p q r]);
jacU = jacobian(Function,[Alph Bet Rcs]);
A = double(subs(jacX,[xSyms,uSyms,cSyms],[xEq,uEq,const]));
B = double(subs(jacU,[xSyms,uSyms,cSyms],[xEq,uEq,const]));



Ka = lqr(A,B,Q,R)







end