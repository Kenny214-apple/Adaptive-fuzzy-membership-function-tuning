%% ============================================================
%  compare_original_vs_ga.m
%  Runs all 5 modes twice — original and GA-optimised FIS
%  Generates comparison table and figures
%% ============================================================
clear; clc;

mode_names  = {'Plowing','Harrowing','Ridging','Planting','Transportation'};
fis_names   = {'tractor_torque_controller.fis', ...
               'tractor_torque_controller_ga.fis'};
ctrl_labels = {'Original Mamdani', 'GA-Optimised GFLS'};

results = struct();

for ctrl = 1:2
    fprintf('\nRunning %s...\n', ctrl_labels{ctrl});

    for i = 1:5
        MODE = i;
        init;

        % Swap FIS file in model programmatically
        mdl = 'electric_tractor';
        blk = [mdl '/Fuzzy Logic Controller'];
        set_param(blk, 'FIS', fis_names{ctrl});

        out = sim(mdl);

        n      = length(out.v_sim);
        ss_idx = max(1, n-9999):n;

        v_ss     = mean(out.v_sim(ss_idx));
        TSR_ss   = mean(out.TSR_sim(ss_idx));
        TSR_pk   = max(out.TSR_sim);
        SoC_drop = (out.SoC_sim(1) - out.SoC_sim(end)) * 100;
        P_avg    = mean(out.P_total_sim);
        d_km     = v_ss * 60 / 1000;
        E_Wh     = P_avg * 60 / 3600;
        Wh_km    = E_Wh / max(d_km, 1e-6);

        % Settling time
        v_ref  = v_ss;
        tout   = out.tout;
        v_data = out.v_sim;
        settled_idx = find(abs(v_data - v_ref) <= 0.02*v_ref & tout > 0.1, 1);
        t_settle = tout(settled_idx);

        % ITAE
        ITAE = trapz(tout, tout .* abs(v_ref - v_data));

        results(ctrl, i).name     = mode_names{i};
        results(ctrl, i).v_ss     = v_ss;
        results(ctrl, i).TSR_ss   = TSR_ss;
        results(ctrl, i).TSR_pk   = TSR_pk;
        results(ctrl, i).SoC_drop = SoC_drop;
        results(ctrl, i).Wh_km    = Wh_km;
        results(ctrl, i).t_settle = t_settle;
        results(ctrl, i).ITAE     = ITAE;

        fprintf('  Mode %d (%s): Wh/km=%.1f | ITAE=%.3f | t_settle=%.3f\n', ...
                i, mode_names{i}, Wh_km, ITAE, t_settle);
    end
end

%% ── PRINT COMPARISON TABLE ───────────────────────────────
fprintf('\n');
fprintf('%s\n', repmat('=',1,100));
fprintf('  PERFORMANCE COMPARISON — ORIGINAL MAMDANI vs GA-OPTIMISED GFLS\n');
fprintf('%s\n', repmat('=',1,100));
fprintf('%-16s  %-12s  %-12s  %-12s  %-12s  %-12s  %-12s\n', ...
        'Mode','ITAE_orig','ITAE_ga','Wh/km_orig','Wh/km_ga',...
        'tsettle_orig','tsettle_ga');
fprintf('%s\n', repmat('-',1,100));

for i = 1:5
    fprintf('%-16s  %-12.4f  %-12.4f  %-12.1f  %-12.1f  %-12.3f  %-12.3f\n', ...
            mode_names{i}, ...
            results(1,i).ITAE,     results(2,i).ITAE, ...
            results(1,i).Wh_km,    results(2,i).Wh_km, ...
            results(1,i).t_settle, results(2,i).t_settle);
end

% Improvement summary
fprintf('%s\n', repmat('-',1,100));
avg_ITAE_imp   = mean(([results(1,:).ITAE] - [results(2,:).ITAE]) ...
                 ./ [results(1,:).ITAE]) * 100;
avg_Whkm_imp   = mean(([results(1,:).Wh_km] - [results(2,:).Wh_km]) ...
                 ./ [results(1,:).Wh_km]) * 100;
avg_settle_imp = mean(([results(1,:).t_settle] - [results(2,:).t_settle]) ...
                 ./ [results(1,:).t_settle]) * 100;

fprintf('AVERAGE IMPROVEMENT:  ITAE: %.2f%%  |  Wh/km: %.2f%%  |  Settling: %.2f%%\n', ...
        avg_ITAE_imp, avg_Whkm_imp, avg_settle_imp);
fprintf('%s\n', repmat('=',1,100));

%% ── COMPARISON FIGURES ───────────────────────────────────
colors = {'b', 'r'};

% Energy comparison grouped bar chart
figure('Name', 'Energy Comparison', 'Position', [100 100 1000 600]);
Wh_data = [[results(1,:).Wh_km]; [results(2,:).Wh_km]]';
b = bar(Wh_data, 'grouped');
b(1).FaceColor = [0.2 0.4 0.7];
b(2).FaceColor = [0.8 0.2 0.2];
set(gca, 'XTickLabel', mode_names, 'FontSize', 11);
legend(ctrl_labels, 'Location', 'northeast', 'FontSize', 11);
xlabel('Operating Mode', 'FontSize', 12);
ylabel('Energy Consumption (Wh/km)', 'FontSize', 12);
title({'Figure 4.X: Energy Consumption Comparison', ...
       'Original Mamdani vs GA-Optimised Genetic Fuzzy Logic System'}, ...
      'FontSize', 12, 'FontWeight', 'bold');
grid on; box on;

% Add percentage improvement labels
for i = 1:5
    imp = (results(1,i).Wh_km - results(2,i).Wh_km) / results(1,i).Wh_km * 100;
    text(i, max(Wh_data(i,:)) * 1.02, sprintf('%.1f%%↓', imp), ...
         'HorizontalAlignment', 'center', 'FontSize', 9, ...
         'Color', [0 0.5 0], 'FontWeight', 'bold');
end
saveas(gcf, 'Fig_Energy_Comparison_GAvsOrig.png');

% ITAE comparison
figure('Name', 'ITAE Comparison', 'Position', [100 100 1000 500]);
ITAE_data = [[results(1,:).ITAE]; [results(2,:).ITAE]]';
bar(ITAE_data, 'grouped');
set(gca, 'XTickLabel', mode_names, 'FontSize', 11);
legend(ctrl_labels, 'Location', 'northeast');
ylabel('ITAE (m·s)', 'FontSize', 12);
title('Figure 4.X: Speed Tracking ITAE — Original vs GA-Optimised', ...
      'FontSize', 12, 'FontWeight', 'bold');
grid on;
saveas(gcf, 'Fig_ITAE_Comparison.png');

fprintf('\nComparison figures saved.\n');