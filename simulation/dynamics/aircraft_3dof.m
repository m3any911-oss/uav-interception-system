function x_dot = aircraft_3dof(x, u_control, p)
% AIRCRAFT_3DOF
% Simplified longitudinal aircraft model with pitch damping.

%% States
u     = x(1);
w     = x(2);
q     = x(3);
theta = x(4);

%% Inputs
delta_e = u_control(1);
delta_t = u_control(2);

%% Airspeed
V = sqrt(u^2 + w^2);
V = max(V, 0.1);

%% Angle of attack
alpha = atan2(w, u);

%% Aerodynamic coefficients
CL = p.CL0 + p.CL_alpha * alpha + p.CL_delta_e * delta_e;
CD = p.CD0 + p.CD_alpha2 * alpha^2;

% Pitching moment (with pitch damping Cm_q)
Cm = p.Cm0 + p.Cm_alpha * alpha + p.Cm_delta_e * delta_e + p.Cm_q * (q * p.c / (2*V));

%% Dynamic pressure
Q = 0.5 * p.rho * V^2;

%% Aerodynamic forces
L = Q * p.S * CL;
D = Q * p.S * CD;

%% Propulsion
T = delta_t * p.T_max;

%% Pitching moment
M = Q * p.S * p.c * Cm;

%% Equations of motion
u_dot = (T - D*cos(alpha) + L*sin(alpha)) / p.m - q*w - p.g*sin(theta);
w_dot = (-D*sin(alpha) - L*cos(alpha)) / p.m + q*u + p.g*cos(theta);
q_dot = M / p.Iyy;
theta_dot = q;

%% State derivative
x_dot = [u_dot; w_dot; q_dot; theta_dot];

end
