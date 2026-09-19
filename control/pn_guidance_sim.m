% PN_GUIDANCE_LQI_SIM - Accurate Kinetic Interception via Proportional Navigation
clear; clc; close all;

%% 1. Target & Interceptor Initial Conditions
% Interceptor (Talon) initial state
r_I = [0; 0];       % Initial position [X_I, Z_I] (m)
V_I = 20;           % Nominal Speed (m/s)

% Target (Enemy Drone) initial state (flying at 15 m altitude)
r_T = [150; -15];   % Initial position [X_T, Z_T] (m) (Z is negative UP)
V_T = 12;           % Target Speed (m/s)
gamma_T = 0;        % Target heading angle (rad)

%% 2. Guidance & Control Parameters
N = 4.0;            % Proportional Navigation Constant (3 <= N <= 5)
dt = 0.01;          % Time step
tspan_max = 0:dt:15; % Max Simulation time horizon

% Load LQI State-Space Model from AVL
A = [ -0.0740,   0.1576,  -0.7015,  -1.0000;
      -0.3198,  -2.0821,  19.6443,   0.0000;
       0.0597,  -1.2549,  -1.6222,   0.0000;
       0.0000,   0.0000,   1.0000,   0.0000 ];
B = [0.0; -2.12; -14.85; 0.0];
C = [0, 0, 0, 1];

A_aug = [A, zeros(4,1); -C, 0];
B_aug = [B; 0];
Q_aug = diag([0.01, 0.01, 1.0, 150, 100]);
R = 80;

K_aug = lqr(A_aug, B_aug, Q_aug, R);
K_x = K_aug(1:4); 
Ki  = -K_aug(5);

%% 3. Simulation Loop with True Kinetic Integration & CPA Cutoff
x_aircraft = [0; 0; 0; 0]; % Aircraft states: [u; w; q; theta]
int_error = 0;
int_max = 15 * (pi/180);

pos_I_hist = [];
pos_T_hist = [];
theta_ref_hist = [];
t_hist = [];

R_rel_prev = inf;

for i = 1:length(tspan_max)
    t_curr = tspan_max(i);
    
    % Save Positions
    pos_I_hist = [pos_I_hist; r_I(1), -r_I(2)]; % Altitude (+Up)
    pos_T_hist = [pos_T_hist; r_T(1), -r_T(2)];
    t_hist     = [t_hist; t_curr];
    
    % 1. Relative Geometry Calculation
    rel_pos = r_T - r_I;
    R_rel = norm(rel_pos); % Relative distance
    
    % [شرط التوقف الدقيق]: إنهاء المحاكاة عند الوصول لأقرب مسافة اقتراب CPA
    if i > 10 && R_rel > R_rel_prev
        fprintf('====================================================\n');
        fprintf('   SUCCESS: TARGET INTERCEPTED AT t = %.2f seconds!\n', t_curr);
        fprintf('   Interception Position: X = %.2f m, Altitude = %.2f m\n', r_I(1), -r_I(2));
        fprintf('   Miss Distance (CPA): %.2f m\n', R_rel_prev);
        fprintf('====================================================\n');
        break;
    end
    R_rel_prev = R_rel;
    
    % Line of Sight (LOS) Angle lambda
    lambda = atan2(-rel_pos(2), rel_pos(1)); 
    
    % True Flight Path Angle gamma = theta - alpha (where alpha approx w/V_I)
    alpha_deg = atan2(x_aircraft(2), V_I + x_aircraft(1));
    gamma_I   = x_aircraft(4) - alpha_deg;
    
    % Relative Velocities & Closing Speed
    v_I_vec = [V_I * cos(gamma_I); -V_I * sin(gamma_I)];
    v_T_vec = [V_T * cos(gamma_T); -V_T * sin(gamma_T)];
    v_rel   = v_T_vec - v_I_vec;
    
    % LOS Rate (d_lambda / dt)
    lambda_dot = (rel_pos(1)*v_rel(2) - rel_pos(2)*v_rel(1)) / (R_rel^2);
    
    % Closing Velocity Vc
    Vc = - (rel_pos' * v_rel) / R_rel;
    
    % 2. Proportional Navigation Law
    a_n = N * Vc * lambda_dot;
    theta_cmd = lambda + (a_n / 9.81); 
    theta_cmd = max(min(theta_cmd, 18*pi/180), -10*pi/180); % Pitch limits
    theta_ref_hist = [theta_ref_hist; theta_cmd];
    
    % 3. Inner-Loop LQI Control Action
    theta_error = theta_cmd - x_aircraft(4);
    int_error = max(min(int_error + theta_error * dt, int_max), -int_max);
    u_cmd = -K_x * x_aircraft + Ki * int_error;
    u_cmd = max(min(u_cmd, 25*pi/180), -25*pi/180);
    
    % 4. State Update
    dxdt = A * x_aircraft + B * u_cmd;
    x_aircraft = x_aircraft + dxdt * dt;
    
    % Kinematic Integration using True Flight Path Angle
    r_I(1) = r_I(1) + V_I * cos(gamma_I) * dt;
    r_I(2) = r_I(2) - V_I * sin(gamma_I) * dt; % Z down
    
    r_T(1) = r_T(1) + V_T * cos(gamma_T) * dt;
    r_T(2) = r_T(2) - V_T * sin(gamma_T) * dt;
end

% Balance history arrays for plotting
if length(theta_ref_hist) < length(t_hist)
    theta_ref_hist = [theta_ref_hist; theta_ref_hist(end)];
end

%% 4. Plotting Interception Trajectory
figure('Name', 'PN Guidance Interception Test');

subplot(2,1,1);
plot(pos_I_hist(:, 1), pos_I_hist(:, 2), 'b-', 'LineWidth', 2); hold on;
plot(pos_T_hist(:, 1), pos_T_hist(:, 2), 'r--', 'LineWidth', 2);
plot(pos_I_hist(end, 1), pos_I_hist(end, 2), 'k*', 'MarkerSize', 12, 'LineWidth', 2); % Impact point
grid on; xlabel('Downrange [m]'); ylabel('Altitude [m]');
title('PN Guidance Interception Trajectory');
legend('Talon Interceptor', 'Target Drone', 'Impact Point');

subplot(2,1,2);
plot(t_hist, theta_ref_hist*(180/pi), 'k', 'LineWidth', 1.5);
grid on; xlabel('Time [s]'); ylabel('Pitch Command [deg]');
title('Generated Guidance Pitch Commands (\theta_{cmd})');
