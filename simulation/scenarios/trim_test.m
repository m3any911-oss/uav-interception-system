function trim_test()
% TRIM_TEST
% Find steady level flight condition for the 3-DOF model.

clear; clc; close all;

%% Parameters
p = aircraft_params();

%% Desired flight condition
V_target = 20;    % m/s
theta_target = 0; % rad

%% Initial guess for [alpha, delta_e, delta_t]
x0_guess = [deg2rad(5), deg2rad(-2), 0.5];

%% Solve trim using fminsearch
options = optimset('Display', 'iter', 'TolX', 1e-6, 'TolFun', 1e-6);
[x_trim, fval] = fminsearch(@(x) trim_cost(x, V_target, theta_target, p), x0_guess, options);

%% Extract results
alpha_trim = x_trim(1);
delta_e_trim = x_trim(2);
delta_t_trim = x_trim(3);

%% Compute trim state
u_trim = V_target * cos(alpha_trim);
w_trim = V_target * sin(alpha_trim);
q_trim = 0;
theta_trim = theta_target;

x_trim_state = [u_trim; w_trim; q_trim; theta_trim];
u_control_trim = [delta_e_trim; delta_t_trim];

%% Display results
fprintf('\n=== TRIM RESULTS ===\n');
fprintf('Airspeed:        %.2f m/s\n', V_target);
fprintf('Alpha:           %.2f deg\n', rad2deg(alpha_trim));
fprintf('Elevator:        %.2f deg\n', rad2deg(delta_e_trim));
fprintf('Throttle:        %.3f\n', delta_t_trim);
fprintf('Cost (residual): %.6e\n', fval);

%% Verify trim
x_dot_trim = aircraft_3dof(x_trim_state, u_control_trim, p);
fprintf('\n=== STATE DERIVATIVES AT TRIM ===\n');
fprintf('u_dot:     %.6e\n', x_dot_trim(1));
fprintf('w_dot:     %.6e\n', x_dot_trim(2));
fprintf('q_dot:     %.6e\n', x_dot_trim(3));
fprintf('theta_dot: %.6e\n', x_dot_trim(4));

end

function J = trim_cost(x, V_target, theta_target, p)
% Cost function for trim: minimize accelerations

alpha = x(1);
delta_e = x(2);
delta_t = x(3);

u = V_target * cos(alpha);
w = V_target * sin(alpha);
q = 0;
theta = theta_target;

x_state = [u; w; q; theta];
u_control = [delta_e; delta_t];

x_dot = aircraft_3dof(x_state, u_control, p);

J = x_dot(1)^2 + x_dot(2)^2 + x_dot(3)^2;

end
