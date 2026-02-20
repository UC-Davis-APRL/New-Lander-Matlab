function xkp1 = DiscreteDynamics(xk, u)
    Ts = 0.05;
    k1 = LQRDynamics.Dynamics(xk,              u);
    k2 = LQRDynamics.Dynamics(xk + 0.5*Ts*k1,  u);
    k3 = LQRDynamics.Dynamics(xk + 0.5*Ts*k2,  u);
    k4 = LQRDynamics.Dynamics(xk + Ts*k3,      u);

    xkp1 = xk + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end