function [A_fun, B_fun f_fun] = makeDynamicsLinearizer()
% linearize_dynamics_subs
% Returns continuous-time linearization about (x0,u0):
%   f = f(x0,u0)
%   A = df/dx |(x0,u0)
%   B = df/du |(x0,u0)
%
% State x = [pn pe pd vn ve vd Phi Theta Psi p q r]'  (12x1)
% Input u = [T Alph Bet Rcs]'                        (4x1)
% constants = [m g L Ixx Iyy Izz]'                   (6x1)
% Fw = [Fw1 Fw2 Fw3]'                                (3x1)


persistent x u const xdot A B  % symbolic objects cached after first call

if isempty(A)
    %% --------- Build symbolic model once ---------
    % Symbols
    syms pn pe pd vn ve vd Phi Theta Psi p q r real
    x = [pn; pe; pd; vn; ve; vd; Phi; Theta; Psi; p; q; r];

    syms T Alph Bet Rcs real
    u = [T; Alph; Bet; Rcs];

    syms m g L Ixx Iyy Izz real
    const = [m; g; L; Ixx; Iyy; Izz];

    % Rotation Rnb (body -> NED), 3-2-1
    cph = cos(Phi);   sph = sin(Phi);
    cth = cos(Theta); sth = sin(Theta);
    cps = cos(Psi);   sps = sin(Psi);

    Rnb = [ cth*cps,  sph*sth*cps - cph*sps,  cph*sth*cps + sph*sps;
            cth*sps,  sph*sth*sps + cph*cps,  cph*sth*sps - sph*cps;
            -sth,     sph*cth,                cph*cth ];

    % Thrust in BODY (your convention)
    Tx_b =  T*sin(Bet)*cos(Alph);
    Ty_b = -T*sin(Alph)*cos(Bet);
    Tz_b = -T*cos(Alph)*cos(Bet);
    F_b  = [Tx_b; Ty_b; Tz_b];

    % Translational accel in NED (gravity down positive)
    a_ned = (Rnb*F_b)/m + [0;0;g];

    % xdot
    xdot = sym(zeros(12,1));
    xdot(1) = vn;
    xdot(2) = ve;
    xdot(3) = vd;

    xdot(4) = a_ned(1);
    xdot(5) = a_ned(2);
    xdot(6) = a_ned(3);

    % Euler kinematics
    xdot(7) = p + sin(Phi)*tan(Theta)*q + cos(Phi)*tan(Theta)*r;
    xdot(8) = cos(Phi)*q - sin(Phi)*r;
    xdot(9) = (sin(Phi)/cos(Theta))*q + (cos(Phi)/cos(Theta))*r;

    % Rotational dynamics
    xdot(10) = (Iyy - Izz)*q*r/Ixx + (L*T*sin(Alph)*cos(Bet))/Ixx;
    xdot(11) = (Izz - Ixx)*p*r/Iyy - (L*T*sin(Bet)*cos(Alph))/Iyy;
    xdot(12) = (Ixx - Iyy)*p*q/Izz + Rcs/Izz;

    % Jacobians
    A = jacobian(xdot, x);
    B = jacobian(xdot, u);
end

%% --------- Substitute numeric operating point ---------
vars = {x, u, const};

f_fun = matlabFunction(xdot, 'Vars', vars);
A_fun = matlabFunction(A,    'Vars', vars);
B_fun = matlabFunction(B,    'Vars', vars);


end