function lin = MakeAttitudeLinearizer()
% Returns function handles lin.A(x,u,C,TEq) and lin.B(x,u,C,TEq)
% where x=[Phi;Theta;Psi;p;q;r], u=[Alph;Bet;Rcs], C=[m g L Ixx Iyy Izz], TEq = T

syms m g L Ixx Iyy Izz T real
syms Phi Theta Psi p q r real
syms Alph Bet Rcs real

% equations
eq7  = p + sin(Phi)*tan(Theta)*q + cos(Phi)*tan(Theta)*r;
eq8  = cos(Phi)*q - sin(Phi)*r;
eq9  = (sin(Phi)/cos(Theta))*q + (cos(Phi)/cos(Theta))*r;
eq10 = (Iyy - Izz)*q*r/Ixx + (L*T*sin(Alph)*cos(Bet))/Ixx;
eq11 = (Izz - Ixx)*p*r/Iyy - (L*T*sin(Bet)*cos(Alph))/Iyy;
eq12 = (Ixx - Iyy)*p*q/Izz + Rcs/Izz;

f = [eq7; eq8; eq9; eq10; eq11; eq12];

xSyms = [Phi Theta Psi p q r];
uSyms = [Alph Bet Rcs];
cSyms = [m g L Ixx Iyy Izz T];

A_sym = jacobian(f, xSyms);
B_sym = jacobian(f, uSyms);

% Make fast numeric evaluators:
A_fun = matlabFunction(A_sym, 'Vars', {xSyms, uSyms, cSyms});
B_fun = matlabFunction(B_sym, 'Vars', {xSyms, uSyms, cSyms});

lin.A = @(x,u,C,TEq) A_fun(x(:).', u(:).', [C(:).', TEq]);
lin.B = @(x,u,C,TEq) B_fun(x(:).', u(:).', [C(:).', TEq]);
end