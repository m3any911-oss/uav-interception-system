% LQI_CONTROLLER_AVL_EXACT - Exact System Realization from MIT AVL
clear; clc; close all;

%% 1. Load Aircraft Parameters
p = aircraft_params();

%% 2. Exact Longitudinal State-Space Matrices (Direct from AVL Output)
% States: x = [u; w; q; theta]  (m/s, m/s, rad/s, rad)
% Input:  u = elevator (rad)

A = [ -0.0740,   0.1576,  -0.7015,  -1.0000;
      -0.3198,  -2.0821,  19.6443,   0.0000;
       0.0597,  -1.2549,  -1.6222,   0.0000;
       0.0000,   0.0000,   1.0000,   0.0000 ];

% B Matrix derived from AVL Elevator Effectiveness
B = [  0.0000;
      -2.1200;
     -14.8500;
       0.0000 ];

C = [0, 0, 0, 1]; % Output: Pitch angle (theta)

%% 3. Augmented System for Integral Action (LQI)
A_aug = [ A,            zeros(4,1);
         -C,            0         ];

B_aug = [ B; 
          0 ];

%% 4. LQI Tuning for Exact Realization
% Penalties: [u, w, q, theta, integral_error]
Q_aug = diag([0.01, 0.01, 1.0, 150, 100]); 
R     = 80; 

% Compute Optimal Control Gains
K_aug = lqr(A_aug, B_aug, Q_aug, R);

K_x = K_aug(1:4);
Ki  = -K_aug(5);

%% 5. Extended Simulation Horizon (60 Seconds Test to Prove Zero Drift)
dt = 0.01;
tspan = 0:dt:60; % 60 Seconds horizon as requested by advisors
theta_ref = 5 * (pi/180); % Target: 5 degrees

x = [0; 0; 0; 0];
int_error = 0;
int_max = 15 * (pi/180); % Anti-windup limit

X_hist = zeros(length(tspan), 4);
U_hist = zeros(length(tspan), 1);

for i = 1:length(tspan)
    % Compute tracking error
    theta_error = theta_ref - x(4);
    
    % Update Integral Action with Anti-Windup
    int_error = int_error + theta_error * dt;
    int_error = max(min(int_error, int_max), -int_max);
    
    % Control Law
    u_cmd = -K_x * x + Ki * int_error;
    
    % Elevator Limits (-25 deg to +25 deg)
    u_cmd = max(min(u_cmd, 25*pi/180), -25*pi/180);
    
    % Dynamics Integration
    dxdt = A * x + B * u_cmd;
    x = x + dxdt * dt;
    
    % Save History
    X_hist(i, :) = x';
    U_hist(i)    = u_cmd;
end

%% 6. Plotting
figure('Name', 'AVL Exact Realization - Extended 60s Simulation');

subplot(3,1,1);
plot(tspan, X_hist(:, 4)*(180/pi), 'b', 'LineWidth', 2); hold on;
yline(5, 'r--', 'LineWidth', 1.5);
grid on; ylabel('Pitch [deg]'); title('Pitch Angle Tracking (Target: 5 deg)');
legend('Actual', 'Reference');

subplot(3,1,2);
plot(tspan, X_hist(:, 3)*(180/pi), 'k', 'LineWidth', 1.2);
grid on; ylabel('q [deg/s]'); title('Pitch Rate');

subplot(3,1,3);
plot(tspan, U_hist*(180/pi), 'm', 'LineWidth', 1.5);
grid on; ylabel('Elevator [deg]'); title('Elevator Control Effort'); xlabel('Time [s]');
