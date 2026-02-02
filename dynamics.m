function xdot = dynamics(x, T, Alph, Bet, Rcs, constants)
% State: [pn pe pd vn ve vd Phi Theta Psi p q r]'

xdot = zeros(12,1);

% constants
c = num2cell(constants);
[m,g,L,Ixx,Iyy,Izz] = deal(c{:});

% unpack state
pn    = x(1); %#ok<NASGU>
pe    = x(2); %#ok<NASGU>
pd    = x(3); %#ok<NASGU>
vn    = x(4);
ve    = x(5);
vd    = x(6);
Phi   = x(7);
Theta = x(8);
Psi   = x(9);
p     = x(10);
q     = x(11);
r     = x(12);

%% 1) Kinematics in NED
xdot(1) = vn;
xdot(2) = ve;
xdot(3) = vd;

%% 2) Build rotation R_nb (body -> NED) consistent with your 3-2-1 angles
cph = cos(Phi);  sph = sin(Phi);
cth = cos(Theta); sth = sin(Theta);
cps = cos(Psi);  sps = sin(Psi);

% R_nb = Rz(Psi)*Ry(Theta)*Rx(Phi)
Rnb = [ cth*cps,  sph*sth*cps - cph*sps,  cph*sth*cps + sph*sps;
        cth*sps,  sph*sth*sps + cph*cps,  cph*sth*sps - sph*cps;
        -sth,     sph*cth,              cph*cth ];

%% 3) Forces -> acceleration in NED

% Thrust direction in BODY.
% If Alph=Bet=0 and thrust is along -body-z, then:
Tx_b =  T*sin(Bet)*cos(Alph);
Ty_b = -T*sin(Alph)*cos(Bet);
Tz_b = -T*cos(Alph)*cos(Bet);
F_b  = [Tx_b; Ty_b; Tz_b];

% Convert to NED and add gravity (down positive)
a_ned = (Rnb * F_b)/m + [0;0;g];

xdot(4) = a_ned(1);
xdot(5) = a_ned(2);
xdot(6) = a_ned(3);

%% 4) Euler angle kinematics (same as yours)
xdot(7) = p + sin(Phi)*tan(Theta)*q + cos(Phi)*tan(Theta)*r;
xdot(8) = cos(Phi)*q - sin(Phi)*r;
xdot(9) = (sin(Phi)/cos(Theta))*q + (cos(Phi)/cos(Theta))*r;

%% 5) Rotational dynamics (same as yours)
xdot(10) = (Iyy - Izz)*q*r/Ixx + (L*T*sin(Alph)*cos(Bet))/Ixx;
xdot(11) = (Izz - Ixx)*p*r/Iyy - (L*T*sin(Bet)*cos(Alph))/Iyy;
xdot(12) = (Ixx - Iyy)*p*q/Izz + Rcs/Izz;

end
