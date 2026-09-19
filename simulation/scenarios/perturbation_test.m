function perturbation_test()
% PERTURBATION_TEST
% Test aircraft response to a small elevator pulse around trim.

clear; clc; close all;

%% Parameters
p = aircraft_params();

%% Desired trim condition
V_target = 20;
theta_target = 0;

%% Find trim
x0_guess = [deg2rad(5), deg2rad(-2), 0.5];
options = optimset('Display', 'off', 'TolX', 1e-6, 'TolFun', 1e-6);
[x_trim, ~] = fminsearch(@(x) trim_cost(x, V_target, theta_target, p), x0_guess, options);

alpha_trim   = x_trim(1);
delta_e_trim = x_trim(2);
delta_t_trim = x_trim(3);

%% Trim state
u_trim = V_target * cos(alpha_trim);
w_trim = V_target * sin(alpha_trim);
q_trim = 0;
theta_trim = theta_target;

x_trim_state = [u_trim; w_trim; q_trim; theta_trim];

%% Perturbation profile
pulse_amplitude = deg2rad(2);   % +2 degrees
pulse_duration  = 0.5;          % seconds

%% Simulation time
tspan = [0 15];

%% Simulate
[t, x] = ode45(@(t,x) dynamics_with_pulse(t, x, p, ...
    delta_e_trim, delta_t_trim, pulse_amplitude, pulse_duration), ...
    tspan, x_trim_state);

%% Extract states
u     = x(:,1);
w     = x(:,2);
q     = x(:,3);
theta = x(:,4);

V     = sqrt(u.^2 + w.^2);
alpha = atan2(w, u);

%% Plot: Airspeed
figure;
plot(t, V, 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('Airspeed [m/s]');
title('Airspeed Response to Elevator Pulse');
yline(V_target, '--r', 'Trim');

%% Plot: Pitch angle
figure;
plot(t, rad2deg(theta), 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('Pitch Angle [deg]');
title('Pitch Angle Response to Elevator Pulse');
yline(rad2deg(theta_target), '--r', 'Trim');

%% Plot: Pitch rate
figure;
plot(t, rad2deg(q), 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('Pitch Rate [deg/s]');
title('Pitch Rate Response to Elevator Pulse');

%% Plot: Angle of attack
figure;
plot(t, rad2deg(alpha), 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('Angle of Attack [deg]');
title('Angle of Attack Response to Elevator Pulse');
yline(rad2deg(alpha_trim), '--r', 'Trim');

%% Plot: Elevator input
delta_e_signal = zeros(size(t));
for i = 1:length(t)
    if t(i) <= pulse_duration
        delta_e_signal(i) = delta_e_trim + pulse_amplitude;
    else
        delta_e_signal(i) = delta_e_trim;
    end
end

figure;
plot(t, rad2deg(delta_e_signal), 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('Elevator [deg]');
title('Elevator Input (Pulse)');

end

%% Dynamics with elevator pulse
function x_dot = dynamics_with_pulse(t, x, p, delta_e_trim, delta_t_trim, pulse_amp, pulse_dur)
    if t <= pulse_dur
        delta_e = delta_e_trim + pulse_amp;
    else
        delta_e = delta_e_trim;
    end
    delta_t = delta_t_trim;
    u_control = [delta_e; delta_t];
    x_dot = aircraft_3dof(x, u_control, p);
end

%% Trim cost function
function J = trim_cost(x, V_target, theta_target, p)
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
