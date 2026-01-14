function K = FindGainMatrix(xEq,uEq,Q,R)
arguments
    xEq 
    uEq 
    % Q matrix for state vector prioritization
    Q (12,12) double = diag([500,500,50,1,1,1,1,1,1,1,1,1]);
    % R matrix for input vector prioritization
    R (4,4) double = diag([10,50,50,4]);
end
%K=FindGainMatrix([0,0,0,0,0,0,0,0,0,0,0,0],[9.81,0,0,0])


% x 
syms pn pe h u v w Phi Theta Psi p q r
xSyms = [pn pe h u v w Phi Theta Psi p q r]


% u 
syms T Alph Bet Rcs
uSyms = [T Alph Bet Rcs]

%constants
const = [1,9.81,1,5,5,5]
syms m g L Ixx Iyy Izz
cSyms = [m g L Ixx Iyy Izz]

%function matrix for xdot, same as nonlinear dynamics
eq1 = cos(Theta)*cos(Psi)*u + (sin(Phi)*sin(Theta)*cos(Psi) - cos(Phi)*sin(Psi))*v + (cos(Phi)*sin(Theta)*cos(Psi) + sin(Phi)*sin(Psi))*w;
eq2 = cos(Theta)*sin(Psi)*u + (sin(Phi)*sin(Theta)*sin(Psi) + cos(Phi)*cos(Psi))*v + (cos(Phi)*sin(Theta)*sin(Psi) - sin(Phi)*cos(Psi))*w;
eq3 = -(sin(Theta)*u - sin(Phi)*cos(Theta)*v - cos(Phi)*cos(Theta)*w);
eq4 = r*v - q*w + (T*sin(Bet)*cos(Alph) - m*g*sin(Theta))/m;
eq5 = p*w - r*u + (-T*sin(Alph)*cos(Bet) + m*g*cos(Theta)*sin(Phi))/m;
eq6 = q*u - p*v + (-T*cos(Alph)*cos(Bet) + m*g*cos(Theta)*cos(Phi))/m;
eq7 = p + sin(Phi)*tan(Theta)*q + cos(Phi)*tan(Theta)*r;
eq8 = cos(Phi)*q - sin(Phi)*r;
eq9 = (sin(Phi)/cos(Theta))*q + (cos(Phi)/cos(Theta))*r;
eq10 = (Iyy - Izz)*q*r/Ixx + (L*T*sin(Alph)*cos(Bet))/Ixx;
eq11 = (Izz - Ixx)*p*r/Iyy - (L*T*sin(Bet)*cos(Alph))/Iyy;
eq12 = (Ixx - Iyy)*p*q/Izz + Rcs/Izz;

Function = [eq1; eq2; eq3; eq4; eq5; eq6; eq7; eq8; eq9; eq10; eq11; eq12];

% jacobians, find symbolic version and then substitute values.
jacX = jacobian(Function,[pn pe h u v w Phi Theta Psi p q r])
jacU = jacobian(Function,[T Alph Bet Rcs])
A = double(subs(jacX,[xSyms,uSyms,cSyms],[xEq,uEq,const]))
B = double(subs(jacU,[xSyms,uSyms,cSyms],[xEq,uEq,const]))



% display(rank(ctrb(A,B)))

K = lqr(A,B,Q,R)


end