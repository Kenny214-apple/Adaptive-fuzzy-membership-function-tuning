%% ================================================
%  BUILD MAMDANI FIS FROM SCRATCH — COMMAND LINE
%  No GUI needed — paste this entire block and run
%% ================================================

% Create Mamdani FIS
fis = mamfis('Name', 'tractor_torque_controller');

% ── INPUT 1: Speed Error (e) ──────────────────────
fis = addInput(fis, [-10 10], 'Name', 'speed_error');
fis = addMF(fis, 'speed_error', 'trimf', [-15 -10 -5],  'Name', 'NL');
fis = addMF(fis, 'speed_error', 'trimf', [-10  -5  0],  'Name', 'NS');
fis = addMF(fis, 'speed_error', 'trimf', [ -5   0  5],  'Name', 'ZE');
fis = addMF(fis, 'speed_error', 'trimf', [  0   5 10],  'Name', 'PS');
fis = addMF(fis, 'speed_error', 'trimf', [  5  10 15],  'Name', 'PL');

% ── INPUT 2: Rate of Change of Error (Δe) ─────────
fis = addInput(fis, [-5 5], 'Name', 'error_rate');
fis = addMF(fis, 'error_rate', 'trimf', [-7.5 -5.0 -2.5], 'Name', 'NL');
fis = addMF(fis, 'error_rate', 'trimf', [-5.0 -2.5  0.0], 'Name', 'NS');
fis = addMF(fis, 'error_rate', 'trimf', [-2.5  0.0  2.5], 'Name', 'ZE');
fis = addMF(fis, 'error_rate', 'trimf', [ 0.0  2.5  5.0], 'Name', 'PS');
fis = addMF(fis, 'error_rate', 'trimf', [ 2.5  5.0  7.5], 'Name', 'PL');

% ── OUTPUT: Torque Split Ratio (TSR) ──────────────
fis = addOutput(fis, [0.40 0.70], 'Name', 'TSR');
fis = addMF(fis, 'TSR', 'trimf', [0.33 0.40 0.48], 'Name', 'VL');
fis = addMF(fis, 'TSR', 'trimf', [0.40 0.48 0.55], 'Name', 'L');
fis = addMF(fis, 'TSR', 'trimf', [0.48 0.55 0.62], 'Name', 'M');
fis = addMF(fis, 'TSR', 'trimf', [0.55 0.62 0.70], 'Name', 'H');
fis = addMF(fis, 'TSR', 'trimf', [0.62 0.70 0.77], 'Name', 'VH');

% ── 25 RULES ──────────────────────────────────────
% Format: [e_mf  de_mf  TSR_mf  weight  connection(1=AND)]
% MF index: NL=1 NS=2 ZE=3 PS=4 PL=5
% TSR index: VL=1 L=2  M=3  H=4  VH=5

ruleList = [
% e    de   TSR  wt  AND
  1    1    1    1   1;   % NL NL → VL
  1    2    1    1   1;   % NL NS → VL
  1    3    1    1   1;   % NL ZE → VL
  1    4    2    1   1;   % NL PS → L
  1    5    2    1   1;   % NL PL → L

  2    1    1    1   1;   % NS NL → VL
  2    2    2    1   1;   % NS NS → L
  2    3    2    1   1;   % NS ZE → L
  2    4    3    1   1;   % NS PS → M
  2    5    3    1   1;   % NS PL → M

  3    1    2    1   1;   % ZE NL → L
  3    2    2    1   1;   % ZE NS → L
  3    3    3    1   1;   % ZE ZE → M
  3    4    4    1   1;   % ZE PS → H
  3    5    4    1   1;   % ZE PL → H

  4    1    3    1   1;   % PS NL → M
  4    2    3    1   1;   % PS NS → M
  4    3    4    1   1;   % PS ZE → H
  4    4    4    1   1;   % PS PS → H
  4    5    5    1   1;   % PS PL → VH

  5    1    3    1   1;   % PL NL → M
  5    2    4    1   1;   % PL NS → H
  5    3    4    1   1;   % PL ZE → H
  5    4    5    1   1;   % PL PS → VH
  5    5    5    1   1;   % PL PL → VH
];

fis = addRule(fis, ruleList);

% ── VERIFY ────────────────────────────────────────
fprintf('\n=== FIS STRUCTURE VERIFICATION ===\n');
fprintf('Inputs  : %d (expected 2)\n', numel(fis.Inputs));
fprintf('Outputs : %d (expected 1)\n', numel(fis.Outputs));
fprintf('Rules   : %d (expected 25)\n', numel(fis.Rules));

% Test key operating points
fprintf('\n=== TSR OUTPUT VERIFICATION ===\n');
test_points = [6.0 3.0; 3.5 1.5; 4.0 2.0; 1.0 0.5; 0.5 0.2];
expected    = [0.654; 0.570; 0.600; 0.510; 0.500];
modes       = {'Plowing','Harrowing','Ridging','Planting','Transport'};

all_pass = true;
for i = 1:5
    TSR_i = evalfis(fis, test_points(i,:));
    pass  = abs(TSR_i - expected(i)) < 0.01;
    if ~pass; all_pass = false; end
    fprintf('%-12s e=%4.1f de=%3.1f → TSR=%.4f (expected %.3f) %s\n', ...
            modes{i}, test_points(i,1), test_points(i,2), ...
            TSR_i, expected(i), char(9989*pass + 10060*(1-pass)));
end

if all_pass
    fprintf('\n✅ All TSR values correct — saving FIS file\n');
    writeFIS(fis, 'tractor_torque_controller.fis');
    fprintf('✅ Saved as tractor_torque_controller.fis\n');
else
    fprintf('\n❌ Some values outside tolerance — check rule list\n');
end