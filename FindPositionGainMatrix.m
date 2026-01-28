function Kp = FindPositionGainMatrix(xEq,uEq,C,Q,R)
arguments
    xEq (1,6) double = [0,0,0,0,0,0]
    uEq (1,3) double = [0,0,0]
    C (1,6) double = [1,9.81,1,5,5,5]
    
    Q (6,6) double = diag([0.2,0.2,0.4,0.5,0.5,0.8]);

    R (3,3) double = diag([50,50,80]);

end
%Kp=FindPositionGainMatrix([0,0,0,0,0,0],[0,0,0,9.81])

%constants
const = C
syms m g L Ixx Iyy Izz
cSyms = [m g L Ixx Iyy Izz]

% x
syms pn pe h u v w
xSyms = [pn pe h u v w]

% u1
syms ax ay az
uSyms = [ax ay az]


%position

eq1 = u;
eq2 = v;
eq3 = w;


eq4 = ax
eq5 = ay
eq6 = az

Function = [eq1; eq2; eq3; eq4; eq5; eq6;];

% jacobians, find symbolic version and then substitute values.
jacX = jacobian(Function,xSyms);
jacU = jacobian(Function,uSyms);
A = double(subs(jacX,[xSyms,uSyms,cSyms],[xEq,uEq,const]))
B = double(subs(jacU,[xSyms,uSyms,cSyms],[xEq,uEq,const]))



Kp = lqr(A,B,Q,R)







end