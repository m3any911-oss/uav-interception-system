function cascade_controller()
% CASCADE_CONTROLLER - Smooth & Stable Pole-Placement Design
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

%% ============ BALANCED CASCADE GAINS ============
% Outer Loop (theta -> q_cmd)
Kp_outer = 1.2;
Ki_outer = 0.1;
Kd_outer = 0.01;

% Inner Loop (q -> delta_e)
Kp_inner = -0.03;
Ki_inner = -0.005;
Kd_inner = 0.00;
% =================================================

%% Simulation setup
dt = 0.01;
T_end = 20;
N = round(T_end / dt);
t = (0:N-1) * dt;

x = zeros(4, N);
x(:,1) = x_trim_state;

delta_e_log     = zeros(1, N);
q_cmd_log       = zeros(1, N);
theta_ref_log   = zeros(1, N);

theta_ref = deg2rad(5);   % Target: 5 degrees step

%% Cascade PID State & Anti-Windup Limits
int_theta_error = 0;
prev_theta_error = 0;

int_q_error = 0;
prev_q_error = 0;

max_int_theta = deg2rad(15);
max_int_q     = deg2rad(15);

%% Simulation Loop
for i = 1:N-1
    % ---- Current State ----
    theta = x(4, i);
    q     = x(3, i);
    
    % ===== OUTER LOOP (theta -> q_cmd) =====
    theta_error = theta_ref - theta;
    int_theta_error = int_theta_error + theta_error * dt;
    int_theta_error = max(min(int_theta_error, max_int_theta), -max_int_theta);
    
    d_theta_error = (theta_error - prev_theta_error) / dt;
    prev_theta_error = theta_error;
    
    q_cmd = Kp_outer*theta_error + Ki_outer*int_theta_error + Kd_outer*d_theta_error;
    q_cmd = max(min(q_cmd, deg2rad(15)), deg2rad(-15)); % Rate Limiter
    
    % ===== INNER LOOP (q -> delta_e) =====
    q_error = q_cmd - q;
    int_q_error = int_q_error + q_error * dt;
    int_q_error = max(min(int_q_error, max_int_q), -max_int_q);
    
    d_q_error = (q_error - prev_q_error) / dt;
    prev_q_error = q_error;
    
    % Control Command Calculation
    delta_e = delta_e_trim + (Kp_inner*q_error + Ki_inner*int_q_error + Kd_inner*d_q_error);
    
    % Actuator Limits
    delta_e = max(min(delta_e, deg2rad(25)), deg2rad(-25));
    
    % ---- Logging ----
    delta_e_log(i)   = delta_e;
    q_cmd_log(i)     = q_cmd;
    theta_ref_log(i) = theta_ref;
    
    % ---- Nonlinear Dynamics Integration ----
    x_dot = aircraft_3dof(x(:,i), [delta_e; delta_t_trim], p);
    x(:,i+1) = x(:,i) + x_dot * dt;
end

delta_e_log(N)   = delta_e_log(N-1);
q_cmd_log(N)     = q_cmd_log(N-1);
theta_ref_log(N) = theta_ref;

%% Extract States
u     = x(1,:);
w     = x(2,:);
q     = x(3,:);
theta = x(4,:);
V     = sqrt(u.^2 + w.^2);

%% ==================== PLOTTING ====================
figure('Name', 'Stable Cascade Controller Results', 'Position', [100, 100, 1200, 800]);

% 1. Pitch Angle Tracking
subplot(2, 3, 1);
plot(t, rad2deg(theta), 'b', 'LineWidth', 1.8); hold on;
plot(t, rad2deg(theta_ref_log), 'r--', 'LineWidth', 1.5);
grid on; xlabel('Time [s]'); ylabel('Pitch Angle [deg]');
title('Pitch Angle Tracking'); legend('Actual', 'Reference');

% 2. Inner Loop: q_cmd vs q
subplot(2, 3, 2);
plot(t, rad2deg(q_cmd_log), 'b--', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(q), 'r', 'LineWidth', 1.5);
grid on; xlabel('Time [s]'); ylabel('Pitch Rate [deg/s]');
title('Inner Loop: q_{cmd} vs q'); legend('q_{cmd}', 'q_{actual}');

% 3. Elevator Command
subplot(2, 3, 3);
plot(t, rad2deg(delta_e_log), 'm', 'LineWidth', 1.5);
grid on; xlabel('Time [s]'); ylabel('Elevator [deg]');
title('Elevator Command');

% 4. Airspeed
subplot(2, 3, 4);
plot(t, V, 'k', 'LineWidth', 1.5);
grid on; xlabel('Time [s]'); ylabel('Airspeed [m/s]');
title('Airspeed');

% 5. Angle of Attack
alpha = atan2(w, u);
subplot(2, 3, 5);
plot(t, rad2deg(alpha), 'c', 'LineWidth', 1.5);
grid on; xlabel('Time [s]'); ylabel('Angle of Attack [deg]');
title('Angle of Attack');

% 6. Performance Metrics
subplot(2, 3, 6); axis off;
theta_final = theta(end);
settling_idx = find(abs(theta - theta_ref) < deg2rad(0.5), 1, 'last');
if ~isempty(settling_idx)
    settling_time = t(settling_idx);
else
    settling_time = NaN;
end
overshoot_val = max(theta) - theta_ref;
if overshoot_val < 0, overshoot_val = 0; end
max_elevator = max(abs(delta_e_log));

text(0.1, 0.9, '=== CASCADE PERFORMANCE ===', 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.75, sprintf('Final Pitch:    %.2f deg', rad2deg(theta_final)), 'FontSize', 11);
text(0.1, 0.6, sprintf('Reference:      %.2f deg', rad2deg(theta_ref)), 'FontSize', 11);
text(0.1, 0.45, sprintf('Settling Time:  %.2f s', settling_time), 'FontSize', 11);
text(0.1, 0.3, sprintf('Overshoot:      %.2f deg', rad2deg(overshoot_val)), 'FontSize', 11);
text(0.1, 0.15, sprintf('Max |Elevator|: %.2f deg', rad2deg(max_elevator)), 'FontSize', 11);

sgtitle('Cascade Controller (Balanced Pole Placement)', 'FontSize', 14, 'FontWeight', 'bold');
end

%% Trim Cost Function
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
