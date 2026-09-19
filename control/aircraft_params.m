function p = aircraft_params()
% AIRCRAFT_PARAMS
% Parameters for X-UAV Talon 1718mm
% Based on Athena Vortex Lattice (AVL) analysis + Corrected CG (Xcg = 0.05m)

%% Physical parameters
p.m   = 2.8;
p.g   = 9.81;
p.Iyy = 0.20;

%% Geometry
p.S = 0.60;
p.c = 0.25;

%% Atmosphere
p.rho = 1.225;

%% Aerodynamic coefficients (Updated from AVL)
p.CL0        = 0.2430;
p.CL_alpha   = 1.1385;    % Updated from AVL (CLa = 1.138461)
p.CL_delta_e = 0.30;
p.CL_q       = 4.8052;    % Added from AVL (CLq = 4.805200)

p.CD0       = 0.00976;
p.CD_alpha2 = 0.01;

%% Pitching moment (STABLE - Updated from AVL with CG @ 0.05m)
p.Cm0        = -0.02;
p.Cm_alpha   = -0.6281;   % Updated from AVL (Cma = -0.628129, Stable Static Margin)
p.Cm_delta_e = -1.2;
p.Cm_q       = -20.2775;  % Updated from AVL (Cmq = -20.277544, High Pitch Damping)

%% Propulsion
p.T_max = 30;

end
