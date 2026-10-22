%% =========================================================================
%  AUTOMATED 5-MODE SIMULATION AND VERIFICATION ROUTINE
%  Project: Modeling & Simulation of a Dual-Motor AWD Electric Tractor
%  File: run_5modes_verification.m
%  Author: Nasirudeen, Ridwan Kehinde (2021/1/81118ET)
%  Department of Mechatronics Engineering, FUT Minna
%  =========================================================================
clear; clc; close all;

%% 1. Model and Global Configuration
modelName = 'electric_tractor';

if ~bdIsLoaded(modelName)
    load_system(modelName);
end

% Set simulation parameters in model configuration
set_param(modelName, 'SolverType', 'Fixed-step');
set_param(modelName, 'Solver', 'ode4');
set_param(modelName, 'FixedStep', '0.001');
set_param(modelName, 'StopTime', '60.0');

mode_names = {'Plowing', 'Harrowing', 'Ridging', 'Planting', 'Transportation'};
num_modes  = length(mode_names);

% Structure to hold extracted verification metrics
results = struct();

fprintf('====================================================================\n');
fprintf(' STARTING AUTOMATED 5-MODE SIMULATION LOOP: %s\n', modelName);
fprintf('====================================================================\n');

%% 2. Iterative 5-Mode Execution Loop
for m = 1:num_modes
    fprintf('--> Initializing and Simulating Mode %d: %s ...\n', m, mode_names{m});
    
    % Set active mode index and evaluate workspace parameters
    MODE = m;
    assignin('base', 'MODE', MODE);
    evalin('base', 'init;'); % Run parameter initialisation script
    
    % Execute Simulink simulation
    simOut = sim(modelName);
    
    % Extract logged signals from Workspace / SimulationOutput
    t          = simOut.tout;
    v_sim      = simOut.out.v_sim.Data;           % Longitudinal speed (m/s)
    tsr_sim    = simOut.out.TSR_out.Data;         % Dynamic Torque Split Ratio
    T_rear     = simOut.out.T_rear_sim.Data;      % Rear motor torque (Nm)
    T_front    = simOut.out.T_front_sim.Data;     % Front motor torque (Nm)
    P_elec     = simOut.out.P_total_sim.Data;     % Instantaneous power (W)
    soc_sim    = simOut.out.SoC_sim.Data * 100;   % Battery SoC (%)
    
    % Retrieve target speed for current mode
    v_target   = evalin('base', 'v_nominal');
    
    % ---------------------------------------------------------------------
    % Performance Metric Computations
    % ---------------------------------------------------------------------
    % Final Steady-State Values (averaged over the last 5 seconds)
    idx_ss       = (t >= 55.0);
    v_ss         = mean(v_sim(idx_ss));
    tsr_ss       = mean(tsr_sim(idx_ss));
    T_rear_ss    = mean(T_rear(idx_ss));
    T_front_ss   = mean(T_front(idx_ss));
    T_total_ss   = T_rear_ss + T_front_ss;
    P_avg_W      = mean(P_elec(idx_ss));
    P_avg_kW     = P_avg_W / 1000;
    
    % Dynamic Transient Metrics
    tsr_peak     = max(tsr_sim);
    speed_err_ss = abs(v_ss - v_target) / v_target * 100;
    
    % Settling Time (time to remain within +/-2% of target speed)
    tol = 0.02 * v_target;
    outside_idx = find(abs(v_sim - v_target) > tol);
    if isempty(outside_idx)
        settling_time = 0;
    else
        last_outside = outside_idx(end);
        if last_outside < length(t)
            settling_time = t(last_outside + 1);
        else
            settling_time = t(end);
        end
    end
    
    % Energy & Battery Autonomy Metrics
    delta_soc    = soc_sim(1) - soc_sim(end);
    total_dist_m = trapz(t, v_sim);
    total_dist_km= total_dist_m / 1000;
    total_E_Wh   = trapz(t, P_elec) / 3600;
    spec_energy  = total_E_Wh / total_dist_km; % Specific energy (Wh/km)
    
    % Estimated endurance (minutes to reach 80% Depth of Discharge)
    endurance_min = (80.0 / delta_soc) * (60.0 / 60.0);
    
    % Store metrics in structured array
    results(m).Mode          = mode_names{m};
    results(m).TargetSpeed   = v_target;
    results(m).SimSpeed      = v_ss;
    results(m).SpeedError    = speed_err_ss;
    results(m).SettlingTime  = settling_time;
    results(m).TSR_Peak      = tsr_peak;
    results(m).TSR_SS        = tsr_ss;
    results(m).T_rear_ss     = T_rear_ss;
    results(m).T_front_ss    = T_front_ss;
    results(m).T_total_ss    = T_total_ss;
    results(m).P_avg_kW      = P_avg_kW;
    results(m).DeltaSoC      = delta_soc;
    results(m).TotalDist_km  = total_dist_km;
    results(m).Total_E_Wh    = total_E_Wh;
    results(m).SpecificEnergy= spec_energy;
    results(m).Endurance_min = endurance_min;
    
    % Save time-history data for post-processing and figure plotting
    results(m).time    = t;
    results(m).v       = v_sim;
    results(m).tsr     = tsr_sim;
    results(m).soc     = soc_sim;
    results(m).P_elec  = P_elec;
end

%% 3. Display Consolidated Performance Table
fprintf('\n========================================================================================================================\n');
fprintf(' %-14s | %-8s | %-8s | %-8s | %-8s | %-8s | %-10s | %-12s | %-10s\n', ...
    'Mode', 'v_tgt(m/s)', 'v_ss(m/s)', 'TSR_pk', 'TSR_ss', 'P_avg(kW)', 'ΔSoC(%)', 'Wh/km', 'Endur(min)');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
for m = 1:num_modes
    fprintf(' %-14s | %10.3f | %10.3f | %8.4f | %8.4f | %9.2f | %8.2f%% | %12.1f | %10.1f\n', ...
        results(m).Mode, results(m).TargetSpeed, results(m).SimSpeed, ...
        results(m).TSR_Peak, results(m).TSR_SS, results(m).P_avg_kW, ...
        results(m).DeltaSoC, results(m).SpecificEnergy, results(m).Endurance_min);
end
fprintf('========================================================================================================================\n');

%% 4. Save Aggregated Simulation Dataset
save('simulation_5modes_results.mat', 'results');
fprintf('\nVerification complete. Summary saved to: simulation_5modes_results.mat\n');