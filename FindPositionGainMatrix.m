function Kp = FindPositionGainMatrix(xEq,uEq,Q,R)
arguments
    xEq 
    uEq
    % Q matrix for state vector prioritization
    Q (6,6) double = diag([1,1,1,1,1,1]);
    % R matrix for input vector prioritization
    R (4,4) double = diag([1,1,1,1]);
    % Q matrix for state vector prioritizatio

end
%Kp=FindPositionGainMatrix([0,0,0,0,0,0],[0,0,0,9.81])

%constants
const = [1,9.81,1,5,5,5]
syms m g L Ixx Iyy Izz
cSyms = [m g L Ixx Iyy Izz]

% x
syms pn pe h u v w
xSyms = [pn pe h u v w]

% u1
syms Phi Theta Psi T
uSyms = [Phi Theta Psi T]

%frozen
frozen = [0,0,0,0,0,0]
syms p q r alph bet rcs
fSyms = [p q r alph bet rcs]

%position
eq1 = cos(Theta)*cos(Psi)*u + (sin(Phi)*sin(Theta)*cos(Psi) - cos(Phi)*sin(Psi))*v + (cos(Phi)*sin(Theta)*cos(Psi) + sin(Phi)*sin(Psi))*w;
eq2 = cos(Theta)*sin(Psi)*u + (sin(Phi)*sin(Theta)*sin(Psi) + cos(Phi)*cos(Psi))*v + (cos(Phi)*sin(Theta)*sin(Psi) - sin(Phi)*cos(Psi))*w;
eq3 = -(sin(Theta)*u - sin(Phi)*cos(Theta)*v - cos(Phi)*cos(Theta)*w);
eq4 = r*v - q*w + (T*sin(bet)*cos(alph) - m*g*sin(Theta))/m;
eq5 = p*w - r*u + (-T*sin(alph)*cos(bet) + m*g*cos(Theta)*sin(Phi))/m;
eq6 = q*u - p*v + (-T*cos(alph)*cos(bet) + m*g*cos(Theta)*cos(Phi))/m;

Function = [eq1; eq2; eq3; eq4; eq5; eq6;];

% jacobians, find symbolic version and then substitute values.
jacX = jacobian(Function,xSyms);
jacU = jacobian(Function,uSyms);
A = double(subs(jacX,[xSyms,uSyms,cSyms,fSyms],[xEq,uEq,const,frozen]));
B = double(subs(jacU,[xSyms,uSyms,cSyms,fSyms],[xEq,uEq,const,frozen]));



Kp = lqr(A,B,Q,R)







end