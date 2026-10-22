%% ============================================================
%  init.m — FYP Parameter Initialisation Script
%  Dual-Motor AWD Electric Tractor Drivetrain with Fuzzy Logic
%  Nasirudeen, R.K. | 2021/1/81118ET | FUT Minna | 2025/2026
%% ============================================================

%% ──────────────────────────────────────────────────────────
%  SECTION 1: VEHICLE PARAMETERS (Derived from MF375 Baseline)
%  ──────────────────────────────────────────────────────────
m        = 2470;      % Total Tractor mass (kg) — Table 3.x
r_wheel  = 0.72;      % Loaded rolling radius (m) — Table 3.x (16.9/14-30 tire)
A        = 2.5;       % Frontal area (m²) — Estimate
C_d      = 0.80;      % Aerodynamic drag coefficient — Typical tractor
rho_air  = 1.225;     % Air density (kg/m³) — Standard ISO
g        = 9.81;      % Gravitational acceleration (m/s²)

%% ──────────────────────────────────────────────────────────
%  SECTION 2: TRANSMISSION / DRIVE LINE
%  ──────────────────────────────────────────────────────────
G_r       = 30;       % Fixed single-stage gear reduction ratio
eta_drive = 0.95;     % Drivetrain efficiency — Combined gear/bearing losses

%% ────────────────────────────────────────────────────────=
%  SECTION 3: MOTOR PARAMETERS (Each of the Dual Motors)
%  ────────────────────────────────────────────────────────=
P_rated   = 15000;    % Rated power per motor (W) — Dual 15 kW PMSM
T_rated   = 200;      % Rated torque per motor (Nm) — Peak Capability
tau_m     = 0.05;     % Motor electromechanical time constant (s)
eta_motor = 0.90;     % Motor efficiency (Steady state)
P_poles   = 6;        % Number of pole pairs
lambda_f  = 0.15;     % Permanent magnet flux linkage (Wb)

%% ──────────────────────────────────────────────────────────
%  SECTION 4: BATTERY MODEL & STATE-OF-CHARGE (SoC)
%  ──────────────────────────────────────────────────────────
V_bus     = 96;       % Nominal DC bus voltage (V)
Q_bat     = 100;      % Battery capacity (Ah)
Q_bat_As  = Q_bat * 3600;  % Total Coulomb Capacity (As) — used in Integration
SoC_0     = 1.0;      % Initial State of Charge (100%)
SoC_min   = 0.20;     % Minimum SoC cutoff (20%)

%% ──────────────────────────────────────────────────────────
%  SECTION 5: PID CONTROLLER GAINS (AWD Speed Regulator)
%  ──────────────────────────────────────────────────────────
%  Gains tuned using Ziegler-Nichols (Z-N) open-loop method
Kp = 3.90;
Ki = 10.83;
Kd = 0.35;

%% ──────────────────────────────────────────────────────────
%  SECTION 6: FUZZY LOGIC CONTROLLER (FLC) PARAMETERS
%  ──────────────────────────────────────────────────────────
%  Universe of discourse (UoD) and scaling factors for Mamdani FIS
e_min   = -10;   e_max   = +10;   % Speed error range (e_m, shaft rpm)
de_min  =  -5;   de_max  =  +5;   % Rate of Error change range (delta_e, rpm/s)
TSR_min =  0.40; TSR_max =  0.70; % Defuzzified Torque split ratio range

%% ──────────────────────────────────────────────────────────
%  SECTION 7: OPERATING MODE SELECTION SWITCH
%  ──────────────────────────────────────────────────────────
%   Configure before running the comparative verification loop
%
%   MODE = 1  →  Plowing (Heavy Primary Tillage)
%   MODE = 2  →  Harrowing (Secondary Tillage)
%   MODE = 3  →  Ridging (Bed / Ridge Formation)
%   MODE = 4  →  Planting / Sowing
%   MODE = 5  →  Transportation (Road Haulage)

if ~exist('MODE', 'var')
    MODE = 1; % Default to Plowing if init.m is run independently
end

% SWITCH CASE STRUCTRUE FOR MODE-SPECIFIC PARAMETERS
switch MODE
    case 1  % --- MODE 1: PLOWING ---
        % Target: 3.0 km/h, deep moldboard plow draught on loose soil
        v_ref       = 3.0 / 3.6; % Convert km/h to m/s (0.833 m/s)
        mu_rr       = 0.25;      % High rolling resistance for tilled soil
        theta_deg   = 5;         % 5-degree field slope
        sigma       = 35000;     % Specific soil resistance (N/m²)
        d_impl      = 0.20;      % Tillage depth (m)
        w_impl      = 0.55;      % Implement working width (m)
        mode_label  = 'Plowing';

    case 2  % --- MODE 2: HARROWING ---
        % Target: 4.0 km/h, disc harrow draught on medium soil
        v_ref       = 4.0 / 3.6; % (1.111 m/s)
        mu_rr       = 0.18;      % Rolling resistance medium soil
        theta_deg   = 2;         % 2-degree slope
        sigma       = 20000;
        d_impl      = 0.10;
        w_impl      = 1.20;
        mode_label  = 'Harrowing';

    case 3  % --- MODE 3: RIDGING ---
        % Target: 4.0 km/h, disc ridger on loose loosened topsoil
        v_ref       = 4.0 / 3.6; % (1.111 m/s)
        mu_rr       = 0.20;      % Rolling resistance loosened soil
        theta_deg   = 3;         % 3-degree slope
        sigma       = 30000;
        d_impl      = 0.15;
        w_impl      = 0.60;
        mode_label  = 'Ridging';

    case 4  % --- MODE 4: PLANTING / SOWING ---
        % Target: 2.5 km/h, light draught planter on prepared seedbed
        v_ref       = 2.5 / 3.6; % (0.694 m/s)
        mu_rr       = 0.15;      % Rolling resistance seedbed
        theta_deg   = 1;         % Flat-to-1 degree slope
        sigma       = 6250;      % Light draught
        d_impl      = 0.04;      % Shallow depth
        w_impl      = 0.80;
        mode_label  = 'Planting';

    case 5  % --- MODE 5: TRANSPORTATION ---
        % Target: 12.0 km/h, road haulage with zero implement draught
        v_ref       = 12.0 / 3.6; % (3.333 m/s)
        mu_rr       = 0.04;       % Low rolling resistance asphalt/farm road
        theta_deg   = 0;          % Flat terrain
        sigma       = 0;          % No implement
        d_impl      = 0;
        w_impl      = 0;
        mode_label  = 'Transportation';
end

%% ──────────────────────────────────────────────────────────
%  SECTION 8: DERIVED DYNAMIC PARAMETERS
%  ──────────────────────────────────────────────────────────
% Convert slope angle to radians
theta_rad   = deg2rad(theta_deg);

% Linear Rolling Resistance Force (N)
F_roll      = mu_rr * m * g * cos(theta_rad);

% Grade/Gravity Resistance Force (N)
F_grade     = m * g * sin(theta_rad);

% Total Agricultural Implement Draught Force (N)
F_implement = sigma * d_impl * w_impl;

% Total Static Terrain Resistance (N) - used for steady-state analysis
F_resistance_total = F_roll + F_grade + F_implement;

% Calculate Aerodynamic factor (k_drag) for quadratic drag force
% Used in Simulink Vehicle Dynamics block: F_drag = k_drag * v^2
k_drag = 0.5 * rho_air * A * C_d;

% Reference motor shaft angular speed (rad/s) needed for PID summing junction
omega_shaft_ref = v_ref * (G_r / r_wheel);

%% ──────────────────────────────────────────────────────────
%  SECTION 9: SIMULINK / VERIFICATION SETTINGS
%  ──────────────────────────────────────────────────────────
out_filename = 'sim_scope_out'; % Scope logging variable name
t_sim   = 60;       % Full simulation duration per mode (s)
dt      = 0.001;    % Simulation fixed timestep (s)

%% ──────────────────────────────────────────────────────────
%  SECTION 10: CONFIRMATION PRINTOUT
%  ──────────────────────────────────────────────────────────
fprintf('\n========================================\n');
fprintf('  FYP MODEL INIT | Nasirudeen R.K. | 2025\n');
fprintf('%s\n', repmat('-', 1, 40));
fprintf('  OPERATING MODE  : %d — %s\n', MODE, mode_label);
fprintf('%s\n', repmat('-', 1, 40));
fprintf('  Reference Speed : %.3f m/s (%.1f km/h)\n', v_ref, v_ref*3.6);
fprintf('  Slope Angle     : %d degrees\n', theta_deg);
fprintf('  Implement Drag  : %.1f N\n', F_implement);
fprintf('  Static Load     : %.1f N\n', F_resistance_total);
fprintf('========================================\n\n');