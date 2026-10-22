%% ============================================================
%  CHAPTER FOUR ALL-FIGURES GENERATION SCRIPT
%  Dual-Motor AWD Electric Tractor — Massey Ferguson 375 Baseline
%  Nasirudeen, R.K. | FUT Minna | 2025/2026
%% ============================================================

clearvars ts_results
mode_names  = {'Plowing','Harrowing','Ridging','Planting','Transportation'};
mode_colors = {'#C0392B','#E67E22','#F1C40F','#27AE60','#2980B9'};
mode_speeds = [0.833, 1.111, 1.111, 0.694, 3.333];

% ── 1. RUN SIMULATION ACROSS ALL 5 MODES ───────────────────────
for i = 1:5
    MODE = i;
    init;
    out = sim('electric_tractor');
    
    % Handle timeseries or raw arrays
    if isa(out.v_sim, 'timeseries')
        v_raw   = out.v_sim.Data;
        SoC_raw = out.SoC_sim.Data;
        TSR_raw = out.TSR_out.Data;
        T_f_raw = out.T_front_sim.Data;
        T_r_raw = out.T_rear_sim.Data;
        P_raw   = out.P_total_sim.Data;
        t_raw   = out.v_sim.Time;
    else
        v_raw   = out.v_sim;
        SoC_raw = out.SoC_sim;
        TSR_raw = out.TSR_out;
        T_f_raw = out.T_front_sim;
        T_r_raw = out.T_rear_sim;
        P_raw   = out.P_total_sim;
        t_raw   = out.tout;
    end
    
    ts_results(i).v     = v_raw;
    ts_results(i).SoC   = SoC_raw;
    ts_results(i).TSR   = TSR_raw;
    ts_results(i).T_f   = T_f_raw;
    ts_results(i).T_r   = T_r_raw;
    ts_results(i).P     = P_raw;
    ts_results(i).t     = t_raw;
    ts_results(i).v_ref = v_ref;
    ts_results(i).name  = mode_names{i};
    
    % Compute Metrics
    N = length(v_raw);
    ss_idx = max(1, round(0.8 * N)) : N;
    v_ss   = mean(v_raw(ss_idx));
    P_avg  = mean(P_raw);
    d_km   = v_ss * 60 / 1000;
    E_Wh   = P_avg * 60 / 3600;
    
    ts_results(i).SoC_drop = (SoC_raw(1) - SoC_raw(end)) * 100;
    ts_results(i).Wh_km    = E_Wh / d_km;
end

t = ts_results(1).t;

%% ── FIGURE 1: Speed Tracking — All 5 Modes ─────────────────
figure('Name','Fig1 Speed Tracking','Position',[100 100 1200 800],'Color','w');
for i = 1:5
    subplot(2,3,i);
    plot(t, ones(size(t))*ts_results(i).v_ref, '--k', ...
         'LineWidth', 1.5, 'DisplayName', 'Reference');
    hold on;
    plot(t, ts_results(i).v, 'Color', mode_colors{i}, ...
         'LineWidth', 2, 'DisplayName', 'Actual');
    hold off;
    xlabel('Time (s)', 'FontSize', 10); ylabel('Speed (m/s)', 'FontSize', 10);
    title(ts_results(i).name, 'FontWeight', 'bold');
    legend('Location','southeast','FontSize',8);
    grid on; box on;
    ylim([0, ts_results(i).v_ref * 1.3]);
    xlim([0 60]);
end
subplot(2,3,6); axis off;
saveas(gcf, 'Fig1_Speed_Tracking.png');
fprintf('Figure 1 saved: Fig1_Speed_Tracking.png\n');

%% ── FIGURE 2: TSR Response — All 5 Modes ───────────────────
figure('Name','Fig2 TSR Response','Position',[100 100 1200 800],'Color','w');
for i = 1:5
    subplot(2,3,i);
    plot(t, ts_results(i).TSR, 'Color', mode_colors{i}, 'LineWidth', 2);
    yline(0.55, '--k', 'Steady-state (0.55)', 'LineWidth', 1.2);
    xlabel('Time (s)', 'FontSize', 10); ylabel('TSR', 'FontSize', 10);
    title(ts_results(i).name, 'FontWeight', 'bold');
    grid on; box on;
    ylim([0.40 0.75]);
    xlim([0 60]);
end
subplot(2,3,6); axis off;
saveas(gcf, 'Fig2_TSR_Response.png');
fprintf('Figure 2 saved: Fig2_TSR_Response.png\n');

%% ── FIGURE 3: SoC Depletion — All Modes on One Plot ────────
figure('Name','Fig3 SoC Depletion','Position',[100 100 900 600],'Color','w');
hold on;
for i = 1:5
    plot(t, ts_results(i).SoC * 100, 'Color', mode_colors{i}, ...
         'LineWidth', 2.5, 'DisplayName', ts_results(i).name);
end
yline(20, '--r', 'Cutoff Limit (20%)', 'LineWidth', 1.5);
hold off;
xlabel('Time (s)', 'FontSize', 12);
ylabel('State of Charge (%)', 'FontSize', 12);
title('Battery State of Charge Depletion Across Operating Modes', 'FontWeight', 'bold');
legend('Location', 'southwest', 'FontSize', 10);
grid on; box on;
xlim([0 60]); ylim([95 101]);
saveas(gcf, 'Fig3_SoC_Depletion.png');
fprintf('Figure 3 saved: Fig3_SoC_Depletion.png\n');

%% ── FIGURE 4: Energy Consumption Bar Chart ──────────────────
Wh_km_vals = [ts_results(1:5).Wh_km];
figure('Name','Fig4 Energy Comparison','Position',[100 100 900 600],'Color','w');
bar_colors = [0.75 0.12 0.12; 0.90 0.42 0.04; ...
              0.95 0.77 0.06; 0.15 0.68 0.38; 0.16 0.50 0.73];
b = bar(Wh_km_vals, 'FaceColor', 'flat');
b.CData = bar_colors;
set(gca, 'XTickLabel', mode_names, 'FontSize', 11);
xlabel('Operating Mode', 'FontSize', 12);
ylabel('Energy Consumption (Wh/km)', 'FontSize', 12);
title('Specific Energy Consumption Comparison (Wh/km)', 'FontWeight', 'bold');
grid on; box on;
for i = 1:5
    text(i, Wh_km_vals(i) + 60, sprintf('%.1f', Wh_km_vals(i)), ...
         'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end
ylim([0 max(Wh_km_vals)*1.18]);
saveas(gcf, 'Fig4_Energy_Comparison.png');
fprintf('Figure 4 saved: Fig4_Energy_Comparison.png\n');

%% ── FIGURE 5: Motor Torque Response — Plowing Mode ─────────
figure('Name','Fig5 Torque Response Plowing','Position',[100 100 900 600],'Color','w');
plot(t, ts_results(1).T_f, 'b-', 'LineWidth', 2, 'DisplayName', 'Front Motor');
hold on;
plot(t, ts_results(1).T_r, 'r-', 'LineWidth', 2, 'DisplayName', 'Rear Motor');
plot(t, ts_results(1).T_f + ts_results(1).T_r, 'k--', ...
     'LineWidth', 1.5, 'DisplayName', 'T_{total}');
yline(200, ':r', 'Rated Limit (200 Nm)', 'LineWidth', 1.2);
hold off;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Motor Torque (Nm)', 'FontSize', 12);
title('Motor Torque Allocation During Plowing Mode', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 10);
grid on; box on;
xlim([0 60]); ylim([0 450]);
saveas(gcf, 'Fig5_Torque_Plowing.png');
fprintf('Figure 5 saved: Fig5_Torque_Plowing.png\n');

%% ── FIGURE 6: Torque Comparison — All Modes Bar Chart ───────
T_front_ss = zeros(1,5);
T_rear_ss  = zeros(1,5);
for i = 1:5
    n = length(ts_results(i).T_f);
    ss_idx = max(1, round(0.8 * n)) : n;
    T_front_ss(i) = mean(ts_results(i).T_f(ss_idx));
    T_rear_ss(i)  = mean(ts_results(i).T_r(ss_idx));
end
figure('Name','Fig6 Torque Comparison','Position',[100 100 900 600],'Color','w');
bar_data = [T_front_ss; T_rear_ss]';
b2 = bar(bar_data, 'grouped');
b2(1).FaceColor = [0.16 0.50 0.73];
b2(2).FaceColor = [0.75 0.12 0.12];
set(gca, 'XTickLabel', mode_names, 'FontSize', 11);
xlabel('Operating Mode', 'FontSize', 12);
ylabel('Steady-State Motor Torque (Nm)', 'FontSize', 12);
title('Front vs. Rear Motor Steady-State Torque Allocation', 'FontWeight', 'bold');
legend({'Front Motor (45%)','Rear Motor (55%)'}, 'Location', 'northeast', 'FontSize', 11);
grid on; box on;
yline(200, '--k', 'Rated Limit per Motor', 'LineWidth', 1.2);
ylim([0 220]);
saveas(gcf, 'Fig6_Torque_Comparison.png');
fprintf('Figure 6 saved: Fig6_Torque_Comparison.png\n');

%% ── FIGURE 7: Power Consumption Over Time ───────────────────
figure('Name','Fig7 Power Consumption','Position',[100 100 900 600],'Color','w');
hold on;
for i = 1:5
    plot(t, ts_results(i).P / 1000, 'Color', mode_colors{i}, ...
         'LineWidth', 2, 'DisplayName', ts_results(i).name);
end
hold off;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Total Electrical Power (kW)', 'FontSize', 12);
title('Total Drivetrain Electrical Power Demand Over Time', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 10);
grid on; box on;
xlim([0 60]);
saveas(gcf, 'Fig7_Power_Consumption.png');
fprintf('Figure 7 saved: Fig7_Power_Consumption.png\n');

%% ── FIGURE 8: Fuzzy Membership Functions ────────────────────
fis = readfis('tractor_torque_controller.fis');
figure('Name','Fig8 Membership Functions','Position',[100 100 1200 400],'Color','w');
subplot(1,3,1);
plotmf(fis, 'input', 1);
title('Input 1: Speed Error e (rpm)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Speed Error (rpm)'); ylabel('Membership Degree');
grid on;

subplot(1,3,2);
plotmf(fis, 'input', 2);
title('Input 2: Error Rate \Deltae (rpm/s)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Error Rate (rpm/s)'); ylabel('Membership Degree');
grid on;

subplot(1,3,3);
plotmf(fis, 'output', 1);
title('Output: Torque Split Ratio (TSR)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('TSR'); ylabel('Membership Degree');
grid on;

saveas(gcf, 'Fig8_Membership_Functions.png');
fprintf('Figure 8 saved: Fig8_Membership_Functions.png\n');

%% ── FIGURE 9: Fuzzy Rule Surface ────────────────────────────
figure('Name','Fig9 Rule Surface','Position',[100 100 800 650],'Color','w');
gensurf(fis);
xlabel('Speed Error e (rpm)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Error Rate \Deltae (rpm/s)', 'FontSize', 11, 'FontWeight', 'bold');
zlabel('Torque Split Ratio (TSR)', 'FontSize', 11, 'FontWeight', 'bold');
title('Mamdani Fuzzy Inference System 3D Control Surface', 'FontWeight', 'bold');
colormap(parula);
colorbar;
view(45, 30);
saveas(gcf, 'Fig9_Rule_Surface.png');
fprintf('Figure 9 saved: Fig9_Rule_Surface.png\n');

%% ── FIGURE 10: SoC Endurance Horizontal Bar Chart ───────────
soc_drops = [ts_results(1:5).SoC_drop];
endurance_min = (80 ./ soc_drops) * 1.0; % 80% usable depth of discharge (100% -> 20%)

figure('Name','Fig10 Endurance','Position',[100 100 900 500],'Color','w');
barh(endurance_min, 'FaceColor', [0.15 0.68 0.38]);
set(gca, 'YTickLabel', mode_names, 'FontSize', 11);
xlabel('Estimated Operational Endurance (Minutes)', 'FontSize', 12);
title('Estimated Operational Endurance on 9.6 kWh Battery (80% DoD)', 'FontWeight', 'bold');
grid on; box on;
for i = 1:5
    text(endurance_min(i) + 2, i, sprintf('%.1f min (%.1f hrs)', endurance_min(i), endurance_min(i)/60), ...
         'VerticalAlignment', 'middle', 'FontSize', 10, 'FontWeight', 'bold');
end
xlim([0 max(endurance_min)*1.25]);
saveas(gcf, 'Fig10_Endurance.png');
fprintf('Figure 10 saved: Fig10_Endurance.png\n');

%% ── FINAL CONFIRMATION ──────────────────────────────────────
fprintf('\n=========================================\n');
fprintf('  ALL 10 CHAPTER FOUR FIGURES GENERATED  \n');
fprintf('=========================================\n');
fprintf('  Fig1  — Speed tracking (all 5 modes)\n');
fprintf('  Fig2  — TSR response (all 5 modes)\n');
fprintf('  Fig3  — SoC depletion (all modes)\n');
fprintf('  Fig4  — Energy comparison bar chart\n');
fprintf('  Fig5  — Motor torque — plowing\n');
fprintf('  Fig6  — Torque comparison all modes\n');
fprintf('  Fig7  — Power consumption over time\n');
fprintf('  Fig8  — Fuzzy membership functions\n');
fprintf('  Fig9  — Fuzzy rule surface\n');
fprintf('  Fig10 — Operational endurance\n');
fprintf('=========================================\n');
fprintf('  All PNG files successfully saved to folder\n');
fprintf('=========================================\n');