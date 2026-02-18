function xkp1 = DiscreteDynamics(xk, u)
    Ts = 0.05;
    k1 = dynamics(xk,              u);
    k2 = dynamics(xk + 0.5*Ts*k1,  u);
    k3 = dynamics(xk + 0.5*Ts*k2,  u);
    k4 = dynamics(xk + Ts*k3,      u);

    xkp1 = xk + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end