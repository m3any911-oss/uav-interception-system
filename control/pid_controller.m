function pid_controller()
% PID_CONTROLLER
% Design and test a PID controller for pitch angle control.

clear; clc; close all;

%% Parameters
p = aircraft_params();

%% Find trim
V_target = 20;
theta_target = 0;

x0_guess = [deg2rad(5), deg2rad(-2), 0.5];
options = optimset('Display', 'off', 'TolX', 1e-6, 'TolFun', 1e-6);
[x_trim, ~] = fminsearch(@(x) trim_cost(x, V_target, theta_target, p), x0_guess, options);

alpha_trim   = x_trim(1);
delta_e_trim = x_trim(2);
delta_t_trim = x_trim(3);

u_trim = V_target * cos(alpha_trim);
w_trim = V_target * sin(alpha_trim);
q_trim = 0;

x_trim_state = [u_trim; w_trim; q_trim; theta_target];

%% PID Gains
Kp = 1.5;
Ki = 0.3;
Kd = 0.4;

%% Simulation setup
dt = 0.01;             % time step [s]
T_end = 20;            % simulation time [s]
N = round(T_end / dt);
t = (0:N-1) * dt;

x = zeros(4, N);
x(:,1) = x_trim_state;

delta_e_log = zeros(1, N);
theta_ref = deg2rad(5);   % 5 degrees step
theta_ref_log = theta_ref * ones(1, N);

%% PID state
integral_error = 0;
prev_error = 0;

%% Simulation loop (Euler Integration)
for i = 1:N-1
    % Current state
    theta = x(4, i);

    % PID
    error = theta_ref - theta;
    integral_error = integral_error + error * dt;
    derivative_error = (error - prev_error) / dt;
    prev_error = error;

    delta_e = delta_e_trim + Kp*error + Ki*integral_error + Kd*derivative_error;

    % Saturate elevator
    delta_e = max(min(delta_e, deg2rad(25)), deg2rad(-25));

    % Log
    delta_e_log(i) = delta_e;

    % Dynamics
    x_dot = aircraft_3dof(x(:,i), [delta_e; delta_t_trim], p);

    % Euler integration
    x(:,i+1) = x(:,i) + x_dot * dt;
end
delta_e_log(N) = delta_e_log(N-1);

%% Extract states
u     = x(1,:);
w     = x(2,:);
q     = x(3,:);
theta = x(4,:);
V     = sqrt(u.^2 + w.^2);

%% Plot: Pitch Angle Tracking
figure;
plot(t, rad2deg(theta), 'LineWidth', 1.8); hold on;
plot(t, rad2deg(theta_ref_log), '--r', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Pitch Angle [deg]');
title('Pitch Angle Tracking with PID');
legend('Actual', 'Reference');

%% Plot: Pitch Rate
figure;
plot(t, rad2deg(q), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Pitch Rate [deg/s]');
title('Pitch Rate Response');

%% Plot: Elevator Command
figure;
plot(t, rad2deg(delta_e_log), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Elevator [deg]');
title('Elevator Command');

%% Plot: Airspeed
figure;
plot(t, V, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Airspeed [m/s]');
title('Airspeed Response');

%% Performance metrics
theta_final = theta(end);
settling_idx = find(abs(theta - theta_ref) < deg2rad(0.5), 1, 'last');
if ~isempty(settling_idx)
    settling_time = t(settling_idx);
else
    settling_time = NaN;
end
overshoot = max(theta) - theta_ref;
if overshoot < 0, overshoot = 0; end

fprintf('\n=== PID PERFORMANCE ===\n');
fprintf('Final Pitch:      %.2f deg\n', rad2deg(theta_final));
fprintf('Reference:        %.2f deg\n', rad2deg(theta_ref));
fprintf('Settling Time:    %.2f s\n', settling_time);
fprintf('Overshoot:        %.2f deg\n', rad2deg(overshoot));

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
