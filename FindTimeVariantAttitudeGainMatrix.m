function K = FindTimeVariantAttitudeGainMatrix(XA, U, C, Q, R)
% XA: 12xN (or 16xN augmented; uses rows 7:12)
% U : 4x(N-1) actual controls
% C : [m g L Ixx Iyy Izz]
% Returns K: 3x6x(N-1) gains for u_att=[Alph Bet Rcs] on x_att=[Phi Theta Psi p q r]

arguments
    XA double
    U double
    C (6,1) double
    Q (6,6) double = diag([80,80,5,40,40,1])
    R (3,3) double = diag([1,1,1])
end

lin = MakeAttitudeLinearizer();

N = size(XA,2);
K = zeros(3,6,N-1);

for k = 1:N-1
    x_att = XA(7:12,k);                 % [Phi;Theta;Psi;p;q;r]
    u_att = U(2:4,k);                   % [Alph;Bet;Rcs]
    Tk    = U(1,k);                     % thrust used in A,B

    Ak = lin.A(x_att, u_att, C, Tk);
    Bk = lin.B(x_att, u_att, C, Tk);

    % Guard: near Theta=±90°, tan/cos terms blow up.
    if abs(cos(x_att(2))) < 0.1
        % fallback: keep previous K (or skip)
        if k > 1
            K(:,:,k) = K(:,:,k-1);
        else
            K(:,:,k) = zeros(3,6);
        end
        continue;
    end

    K(:,:,k) = lqr(Ak, Bk, Q, R);
end
end
