%% ============================================================
%  genetic_fuzzy_optimizer.m
%  Genetic Algorithm Optimisation of Mamdani FIS Membership
%  Function Parameters for Dual-Motor Electric Tractor
%  Nasirudeen, R.K. | FUT Minna | 2025/2026
%% ============================================================
%
%  WHAT THIS DOES:
%  Optimises the peak positions of all 15 triangular membership
%  functions (5 per variable × 3 variables) using MATLAB's ga()
%  function. The rule base is unchanged. Only MF shapes evolve.
%
%  CHROMOSOME: x (1×15 double)
%  x(1:5)   = peak positions of speed_error MFs  [NL NS ZE PS PL]
%  x(6:10)  = peak positions of error_rate MFs   [NL NS ZE PS PL]
%  x(11:15) = peak positions of TSR output MFs   [VL  L  M  H VH]
%
%  HOW TO RUN:
%  >> genetic_fuzzy_optimizer
%  Takes ~15–30 minutes. Saves result as
%  tractor_torque_controller_ga.fis
%% ============================================================

clear; clc;
fprintf('=================================================\n');
fprintf('  GENETIC FUZZY LOGIC OPTIMISATION\n');
fprintf('  Dual-Motor Electric Tractor Drivetrain\n');
fprintf('=================================================\n\n');

%% ── LOAD BASE PARAMETERS ─────────────────────────────────
% Load vehicle and controller parameters into workspace
% These are used inside the fitness function
global P;
P.m         = 2470;
P.r_wheel   = 0.72;
P.G_r       = 30;
P.eta_drive = 0.95;
P.eta_motor = 0.90;
P.V_bus     = 96;
P.Q_bat_As  = 360000;
P.Kp        = 3.90;
P.Ki        = 10.83;
P.Kd        = 0.35;
P.tau_m     = 0.05;
P.N_filt    = 100;    % PID derivative filter coefficient
P.rho_air   = 1.225;
P.Cd        = 0.80;
P.A_front   = 2.5;
P.g         = 9.81;

% Five operating modes [v_ref, mu_rr, theta_deg, F_implement]
P.modes = [
    0.833,  0.25,  5,  3850;   % Plowing
    1.111,  0.20,  5,  2400;   % Harrowing
    1.111,  0.22,  5,  2700;   % Ridging
    0.694,  0.16,  0,   500;   % Planting
    3.333,  0.12,  0,     0;   % Transportation
];

% Mode weights for fitness (heavier weight to more demanding modes)
P.mode_weights = [0.30, 0.20, 0.20, 0.15, 0.15];

% Fitness weights
P.w_itae      = 0.50;   % Speed tracking error weight
P.w_energy    = 0.30;   % Energy efficiency weight
P.w_overshoot = 0.20;   % Overshoot penalty weight

% Simulation settings
P.dt     = 0.005;    % Timestep (5ms — faster than Simulink for GA)
P.t_end  = 60;       % 60-second simulation per mode
P.t_vec  = 0 : P.dt : P.t_end;
P.N      = length(P.t_vec);

%% ── CHROMOSOME BOUNDS ────────────────────────────────────
% Each peak is allowed to vary ±30% of the initial spacing
% This guarantees ordering is maintained (ranges never overlap)

% speed_error initial peaks: [-10, -5, 0, 5, 10] — spacing 5
% Allow ±1.5 variation per peak
lb_e  = [-11.5, -6.5, -1.5,  3.5,  8.5];
ub_e  = [ -8.5, -3.5,  1.5,  6.5, 11.5];

% error_rate initial peaks: [-5, -2.5, 0, 2.5, 5] — spacing 2.5
% Allow ±0.75 variation per peak
lb_de = [-5.75, -3.25, -0.75,  1.75,  4.25];
ub_de = [-4.25, -1.75,  0.75,  3.25,  5.75];

% TSR output initial peaks: [0.40, 0.48, 0.55, 0.62, 0.70] — spacing ~0.075
% Allow ±0.025 variation per peak
lb_t  = [0.375, 0.455, 0.525, 0.595, 0.675];
ub_t  = [0.425, 0.505, 0.575, 0.645, 0.725];

lb = [lb_e,  lb_de,  lb_t];
ub = [ub_e,  ub_de,  ub_t];

n_vars = 15;

%% ── INITIAL POPULATION SEED ──────────────────────────────
% Seed the GA with the original manually-tuned values as one member
% This guarantees GA can only equal or improve the original design

x0 = [-10, -5, 0, 5, 10, ...         % speed_error peaks (original)
        -5, -2.5, 0, 2.5, 5, ...     % error_rate peaks  (original)
       0.40, 0.48, 0.55, 0.62, 0.70];% TSR peaks         (original)

fprintf('Original (manually tuned) fitness: %.6f\n', ...
        ga_fitness(x0));

%% ── GENETIC ALGORITHM OPTIONS ────────────────────────────
opts = optimoptions('ga', ...
    'PopulationSize',        50, ...       % Number of chromosomes
    'MaxGenerations',        80, ...       % Number of generations
    'EliteCount',             4, ...       % Top individuals preserved
    'CrossoverFraction',    0.80, ...      % Crossover probability
    'MutationFcn',       {@mutationadaptfeasible}, ...
    'SelectionFcn',      @selectiontournament, ...
    'CrossoverFcn',      @crossoverscattered, ...
    'InitialPopulationMatrix', x0, ...    % Seed with original solution
    'FunctionTolerance',  1e-5, ...
    'MaxStallGenerations',   20, ...
    'Display',          'iter', ...        % Show progress each generation
    'PlotFcn',  @gaplotbestf, ...          % Plot convergence in real time
    'UseParallel',         false);         % Set true if Parallel Toolbox

%% ── RUN GENETIC ALGORITHM ────────────────────────────────
fprintf('\nStarting Genetic Algorithm...\n');
fprintf('Population: %d | Generations: %d | Variables: %d\n\n', ...
        50, 80, n_vars);

tic;
[x_opt, f_opt, exitflag, output] = ga(@ga_fitness, n_vars, ...
    [], [], [], [], lb, ub, [], opts);
elapsed = toc;

%% ── DISPLAY RESULTS ──────────────────────────────────────
fprintf('\n=================================================\n');
fprintf('  GA OPTIMISATION COMPLETE\n');
fprintf('=================================================\n');
fprintf('Time elapsed       : %.1f minutes\n', elapsed/60);
fprintf('Generations run    : %d\n', output.generations);
fprintf('Function evals     : %d\n', output.funccount);
fprintf('Original fitness   : %.6f\n', ga_fitness(x0));
fprintf('Optimised fitness  : %.6f\n', f_opt);
fprintf('Improvement        : %.2f%%\n', ...
        (ga_fitness(x0) - f_opt) / ga_fitness(x0) * 100);

fprintf('\nOptimised MF Peak Positions:\n');
fprintf('  speed_error  : [%.3f  %.3f  %.3f  %.3f  %.3f] rpm\n', ...
        x_opt(1), x_opt(2), x_opt(3), x_opt(4), x_opt(5));
fprintf('  error_rate   : [%.3f  %.3f  %.3f  %.3f  %.3f] rpm/s\n', ...
        x_opt(6), x_opt(7), x_opt(8), x_opt(9), x_opt(10));
fprintf('  TSR output   : [%.4f  %.4f  %.4f  %.4f  %.4f]\n', ...
        x_opt(11), x_opt(12), x_opt(13), x_opt(14), x_opt(15));

%% ── BUILD AND SAVE OPTIMISED FIS ─────────────────────────
fis_ga = build_fis(x_opt);
writeFIS(fis_ga, 'tractor_torque_controller_ga.fis');
fprintf('\nSaved: tractor_torque_controller_ga.fis\n');

%% ── COMPARE MF PLOTS ─────────────────────────────────────
fis_orig = readfis('tractor_torque_controller.fis');

figure('Name', 'MF Comparison: Original vs GA-Optimised', ...
       'Position', [100 100 1400 900]);

titles  = {'speed\_error (rpm)', 'error\_rate (rpm/s)', 'TSR output'};
types   = {'input', 'input', 'output'};
idx_arr = {1, 2, 1};

for k = 1:3
    subplot(2, 3, k);
    plotmf(fis_orig, types{k}, idx_arr{k});
    title(['Original — ' titles{k}], 'FontSize', 10);
    grid on;

    subplot(2, 3, k+3);
    plotmf(fis_ga, types{k}, idx_arr{k});
    title(['GA-Optimised — ' titles{k}], 'FontSize', 10);
    grid on;
end

sgtitle('Figure 4.X: Membership Function Comparison — Original Mamdani vs GA-Optimised GFLS', ...
        'FontSize', 12, 'FontWeight', 'bold');
saveas(gcf, 'Fig_MF_Comparison.png');

%% ── COMPARE RULE SURFACES ────────────────────────────────
figure('Name', 'Rule Surface Comparison', 'Position', [100 100 1200 500]);

subplot(1,2,1);
gensurf(fis_orig);
title('Original Mamdani — Rule Surface', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Speed error e (rpm)'); ylabel('Error rate Δe (rpm/s)');
zlabel('TSR'); colorbar; grid on;

subplot(1,2,2);
gensurf(fis_ga);
title('GA-Optimised GFLS — Rule Surface', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Speed error e (rpm)'); ylabel('Error rate Δe (rpm/s)');
zlabel('TSR'); colorbar; grid on;

sgtitle('Figure 4.X: Rule Surface Comparison — Original vs GA-Optimised', ...
        'FontSize', 12, 'FontWeight', 'bold');
saveas(gcf, 'Fig_Rule_Surface_Comparison.png');

fprintf('\nAll figures saved. \n');
fprintf('Now load tractor_torque_controller_ga.fis in Simulink\n');
fprintf('and re-run all 5 modes to get optimised results.\n');


%% ============================================================
%  LOCAL FUNCTION: ga_fitness
%  Evaluates fitness of a chromosome by simulating the
%  closed-loop system in pure MATLAB (no Simulink — fast)
%% ============================================================
function J = ga_fitness(x)
    global P;

    % Build FIS from chromosome
    fis = build_fis(x);

    J_total = 0;

    for mode_idx = 1:5
        % Mode parameters
        v_ref      = P.modes(mode_idx, 1);
        mu_rr      = P.modes(mode_idx, 2);
        theta_deg  = P.modes(mode_idx, 3);
        F_impl     = P.modes(mode_idx, 4);
        theta_rad  = theta_deg * pi / 180;

        % Static resistive forces (pre-computed outside time loop)
        F_roll  = mu_rr * P.m * P.g * cos(theta_rad);
        F_grade = P.m * P.g * sin(theta_rad);

        % Reference shaft speed
        omega_ref = v_ref * P.G_r / P.r_wheel;

        % Convert omega_ref to rpm for fuzzy input
        omega_ref_rpm = omega_ref * 60 / (2*pi);

        % Initialise states
        v        = 0;           % vehicle speed (m/s)
        T_out_f  = 0;           % front motor output torque
        T_out_r  = 0;           % rear motor output torque
        integral = 0;           % PID integral term
        e_prev   = omega_ref;   % previous speed error (rad/s)
        d_filt   = 0;           % filtered derivative
        SoC      = 1.0;         % battery SoC

        % Performance accumulators
        ITAE      = 0;
        E_total   = 0;
        overshoot = 0;
        settled   = false;
        t_settle  = P.t_end;

        % Simulation loop (Euler integration)
        for k = 1:P.N
            t = P.t_vec(k);

            % ── Actual shaft speed ──────────────────────────
            omega_actual     = v * P.G_r / P.r_wheel;
            omega_actual_rpm = omega_actual * 60 / (2*pi);

            % ── Speed error ─────────────────────────────────
            e = omega_ref - omega_actual;         % rad/s

            % ── PID controller (Euler, anti-windup) ─────────
            e_rpm   = e * 60 / (2*pi);            % convert to rpm
            d_raw   = (e - e_prev) / P.dt;
            d_filt  = d_filt + P.dt * P.N_filt * (d_raw - d_filt);

            u       = P.Kp * e + P.Ki * integral + P.Kd * d_filt;
            u_sat   = max(0, min(400, u));         % saturate 0–400 Nm

            % Anti-windup: only integrate if not saturated
            if u == u_sat
                integral = integral + e * P.dt;
            end
            T_total = u_sat;

            % ── Fuzzy controller ────────────────────────────
            de_rpm   = d_filt * 60 / (2*pi);      % error rate in rpm/s

            % Clamp inputs to defined fuzzy ranges
            e_clamp  = max(-10, min(10,  e_rpm));
            de_clamp = max(-5,  min(5,   de_rpm));

            TSR = evalfis(fis, [e_clamp, de_clamp]);
            TSR = max(0.40, min(0.70, TSR));      % hard clamp output

            % ── Torque allocation ───────────────────────────
            T_ref_f = (1 - TSR) * T_total;
            T_ref_r = TSR * T_total;

            % ── Motor dynamics (first-order Euler) ──────────
            T_out_f = T_out_f + P.dt/P.tau_m * (T_ref_f - T_out_f);
            T_out_r = T_out_r + P.dt/P.tau_m * (T_ref_r - T_out_r);

            % Clamp motor outputs to rated torque
            T_out_f = max(0, min(200, T_out_f));
            T_out_r = max(0, min(200, T_out_r));

            % ── Traction force ──────────────────────────────
            F_tract = (T_out_f + T_out_r) * P.G_r * P.eta_drive / P.r_wheel;

            % ── Aerodynamic drag ─────────────────────────────
            F_drag  = 0.5 * P.rho_air * P.Cd * P.A_front * v^2;

            % ── Net force → acceleration → speed ────────────
            F_net = F_tract - F_roll - F_drag - F_grade - F_impl;
            a     = F_net / P.m;
            v     = max(0, v + a * P.dt);

            % ── Power and SoC ────────────────────────────────
            omega_m  = v * P.G_r / P.r_wheel;
            P_mech   = (T_out_f + T_out_r) * omega_m;
            P_elec   = P_mech / P.eta_motor;
            I_bat    = P_elec / P.V_bus;
            SoC      = SoC - I_bat * P.dt / P.Q_bat_As;
            E_total  = E_total + P_elec * P.dt;

            % ── Performance metrics ──────────────────────────
            % ITAE: Integral of Time-weighted Absolute Error
            ITAE = ITAE + t * abs(v_ref - v) * P.dt;

            % Overshoot
            if v > overshoot
                overshoot = v;
            end

            % Settling time (first time within ±2% of reference)
            if ~settled && t > 0.5
                if abs(v - v_ref) <= 0.02 * v_ref
                    t_settle = t;
                    settled  = true;
                end
            end

            e_prev = e;
        end

        % ── Normalise metrics for this mode ─────────────────
        d_km = v_ref * P.t_end / 1000;            % distance (km)
        Wh_km = (E_total / 3600) / max(d_km, 1e-6);

        % Reference values (original manually-tuned FIS)
        Wh_km_ref   = [3813.2, 2461.9, 2847.2, 1379.6, 342.3];
        t_settle_ref = 1.8;   % seconds (approximate)

        % Normalised objectives (0 = perfect, 1 = as bad as reference)
        ITAE_norm      = ITAE / (v_ref * P.t_end^2 / 2);
        energy_norm    = Wh_km / Wh_km_ref(mode_idx);
        overshoot_norm = max(0, (overshoot - v_ref)) / (0.1 * v_ref);
        settle_norm    = t_settle / t_settle_ref;

        J_mode = P.w_itae      * ITAE_norm      + ...
                 P.w_energy    * energy_norm     + ...
                 P.w_overshoot * overshoot_norm  + ...
                 0.10          * settle_norm;

        J_total = J_total + P.mode_weights(mode_idx) * J_mode;
    end

    J = J_total;
end


%% ============================================================
%  LOCAL FUNCTION: build_fis
%  Constructs a Mamdani FIS from a chromosome of 15 peak values
%% ============================================================
function fis = build_fis(x)

    % Helper: build trimf parameters from 5 peaks
    % Feet are set at midpoints between adjacent peaks
    % (outer feet extend proportionally beyond first/last peak)
    function params = peaks_to_trimf(peaks)
        n = length(peaks);
        params = zeros(n, 3);  % [a, b, c] for each MF

        for i = 1:n
            % Left foot
            if i == 1
                spacing = peaks(2) - peaks(1);
                params(i,1) = peaks(i) - spacing;
            else
                params(i,1) = (peaks(i-1) + peaks(i)) / 2;
            end

            % Peak
            params(i,2) = peaks(i);

            % Right foot
            if i == n
                spacing = peaks(n) - peaks(n-1);
                params(i,3) = peaks(i) + spacing;
            else
                params(i,3) = (peaks(i) + peaks(i+1)) / 2;
            end
        end
    end

    % Extract peaks from chromosome
    peaks_e   = x(1:5);
    peaks_de  = x(6:10);
    peaks_tsr = x(11:15);

    % Build trimf parameter matrices
    mf_e   = peaks_to_trimf(peaks_e);
    mf_de  = peaks_to_trimf(peaks_de);
    mf_tsr = peaks_to_trimf(peaks_tsr);

    % Labels
    e_labels   = {'NL','NS','ZE','PS','PL'};
    de_labels  = {'NL','NS','ZE','PS','PL'};
    tsr_labels = {'VL','L','M','H','VH'};

    % Construct FIS
    fis = mamfis('Name', 'tractor_torque_controller_ga', ...
                 'AndMethod',           'min', ...
                 'OrMethod',            'max', ...
                 'ImplicationMethod',   'min', ...
                 'AggregationMethod',   'max', ...
                 'DefuzzificationMethod', 'centroid');

    % Input 1: speed_error
    fis = addInput(fis, [-10 10], 'Name', 'speed_error');
    for i = 1:5
        fis = addMF(fis, 'speed_error', 'trimf', mf_e(i,:), ...
                    'Name', e_labels{i});
    end

    % Input 2: error_rate
    fis = addInput(fis, [-5 5], 'Name', 'error_rate');
    for i = 1:5
        fis = addMF(fis, 'error_rate', 'trimf', mf_de(i,:), ...
                    'Name', de_labels{i});
    end

    % Output: TSR
    fis = addOutput(fis, [0.40 0.70], 'Name', 'TSR');
    for i = 1:5
        fis = addMF(fis, 'TSR', 'trimf', mf_tsr(i,:), ...
                    'Name', tsr_labels{i});
    end

    % 25-rule base (identical to original — only MFs are evolved)
    ruleList = [
        1 1 1 1 1;   % NL NL → VL
        1 2 1 1 1;   % NL NS → VL
        1 3 1 1 1;   % NL ZE → VL
        1 4 2 1 1;   % NL PS → L
        1 5 2 1 1;   % NL PL → L
        2 1 1 1 1;   % NS NL → VL
        2 2 2 1 1;   % NS NS → L
        2 3 2 1 1;   % NS ZE → L
        2 4 3 1 1;   % NS PS → M
        2 5 3 1 1;   % NS PL → M
        3 1 2 1 1;   % ZE NL → L
        3 2 2 1 1;   % ZE NS → L
        3 3 3 1 1;   % ZE ZE → M
        3 4 4 1 1;   % ZE PS → H
        3 5 4 1 1;   % ZE PL → H
        4 1 3 1 1;   % PS NL → M
        4 2 3 1 1;   % PS NS → M
        4 3 4 1 1;   % PS ZE → H
        4 4 4 1 1;   % PS PS → H
        4 5 5 1 1;   % PS PL → VH
        5 1 3 1 1;   % PL NL → M
        5 2 4 1 1;   % PL NS → H
        5 3 4 1 1;   % PL ZE → H
        5 4 5 1 1;   % PL PS → VH
        5 5 5 1 1;   % PL PL → VH
    ];

    fis = addRule(fis, ruleList);
end