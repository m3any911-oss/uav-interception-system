%% Dynamic Pneumatic Actuator Model (Energy & Mass Balance)
clear; clc; close all;

% --- Physical Constants & Gas Properties ---
p.R     = 287.05;      % Air gas constant [J/(kg*K)]
p.gamma = 1.4;         % Specific heat ratio
p.Cp    = 1005;        % Specific heat at constant pressure [J/(kg*K)]
p.Cv_gas= 718;         % Specific heat at constant volume [J/(kg*K)]

% --- Actuator & System Parameters ---
p.A     = 0.0019635;   % Piston Area (D = 50 mm) [m^2]
p.Vacc  = 0.005;       % Accumulator Volume (5 Liters) [m^3]
p.Vdead = 0.0001;      % Dead Volume [m^3]
p.m     = 3.5;         % Drone / Moving Mass [kg]
p.Pback = 101325;      % Atmospheric Back Pressure [Pa]

% --- Valve Parameters ---
p.Av    = 0.0002;      % Valve Effective Area (Equivalent to Cv ~ 3.0) [m^2]
p.Fc    = 15.0;        % Coulomb Friction [N]
p.c_visc= 10.0;        % Viscous Friction Coefficient [N*s/m]

% --- Initial Conditions ---
Pa0 = 8.0e5 + 101325;  % Initial Accumulator Pressure (8 bar gauge) [Pa]
Ta0 = 293.15;          % Initial Temperature (20°C) [K]
ma0 = (Pa0 * p.Vacc) / (p.R * Ta0); % Initial Mass in Accumulator [kg]

Pc0 = 101325;          % Initial Cylinder Pressure (Atmospheric) [Pa]
Tc0 = 293.15;          % Initial Cylinder Temperature [K]
mc0 = (Pc0 * p.Vdead) / (p.R * Tc0); % Initial Mass in Cylinder [kg]

% State vector: y = [ma; Ta; mc; Tc; x; v]
y0 = [ma0; Ta0; mc0; Tc0; 0; 0];
tspan = [0 0.3]; % Simulation time: 300 ms

opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t, y] = ode45(@(t, y) pneumaticEnergyODE(t, y, p), tspan, y0, opts);

% --- Extract & Calculate Plot Outputs ---
ma = y(:,1); Ta = y(:,2);
mc = y(:,3); Tc = y(:,4);
x  = y(:,5); v  = y(:,6);

Pa = (ma .* p.R .* Ta) ./ p.Vacc;
Vc = p.Vdead + p.A .* x;
Pc = (mc .* p.R .* Tc) ./ Vc;

% --- Plotting Results ---
figure('Name', 'Pneumatic System Dynamic Simulation');
subplot(2,2,1);
plot(t*1000, (Pa-101325)/1e5, 'r', 'LineWidth', 1.5); hold on;
plot(t*1000, (Pc-101325)/1e5, 'b', 'LineWidth', 1.5);
xlabel('Time [ms]'); ylabel('Pressure [bar gauge]');
legend('Accumulator (Pa)', 'Cylinder (Pc)'); grid on; title('Pressure Profiles');

subplot(2,2,2);
plot(t*1000, x, 'k', 'LineWidth', 1.5);
xlabel('Time [ms]'); ylabel('Position [m]');
grid on; title('Piston Displacement (x)');

subplot(2,2,3);
plot(t*1000, v, 'm', 'LineWidth', 1.5);
xlabel('Time [ms]'); ylabel('Velocity [m/s]');
grid on; title('Piston Velocity (v)');

subplot(2,2,4);
plot(t*1000, Ta-273.15, 'r--', 'LineWidth', 1.2); hold on;
plot(t*1000, Tc-273.15, 'b--', 'LineWidth', 1.2);
xlabel('Time [ms]'); ylabel('Temperature [°C]');
legend('Accumulator (Ta)', 'Cylinder (Tc)'); grid on; title('Gas Temperatures');

% --- ODE Function ---
function dydt = pneumaticEnergyODE(~, y, p)
    ma = max(y(1), 1e-6); Ta = max(y(2), 100);
    mc = max(y(3), 1e-6); Tc = max(y(4), 100);
    x  = max(y(5), 0);    v  = y(6);

    Pa = (ma * p.R * Ta) / p.Vacc;
    Vc = p.Vdead + p.A * x;
    Pc = (mc * p.R * Tc) / Vc;

    % Flow Regime Calculation
    pratio = Pc / Pa;
    p_crit = (2 / (p.gamma + 1))^(p.gamma / (p.gamma - 1));

    if pratio <= p_crit
        % Choked Flow
        mdot = p.Av * Pa * sqrt(p.gamma / (p.R * Ta)) * (2 / (p.gamma + 1))^((p.gamma + 1) / (2 * (p.gamma - 1)));
    else
        % Subsonic Flow
        term = pratio^(2/p.gamma) - pratio^((p.gamma+1)/p.gamma);
        term = max(term, 0);
        mdot = p.Av * Pa * sqrt((2 * p.gamma) / (p.R * Ta * (p.gamma - 1)) * term);
    end
    mdot = max(mdot, 0);

    % Energy & Mass Differential Equations
    dma = -mdot;
    dTa = (Ta / ma) * (1 - p.gamma) * mdot;

    dmc = mdot;
    dTc = (p.R * Tc / (mc * p.Cv_gas)) * (p.Cp * Ta * mdot - Pc * p.A * v - p.Cv_gas * Tc * mdot);

    % Mechanical Dynamics
    Fp = (Pc - p.Pback) * p.A;
    Ffric = (abs(v) > 1e-4) * (p.Fc * sign(v) + p.c_visc * v);
    accel = (Fp - Ffric) / p.m;

    % Stop at end of rail (1.5m)
    if x >= 1.5 && accel > 0
        v = 0; accel = 0;
    end

    dydt = [dma; dTa; dmc; dTc; v; accel];
end
