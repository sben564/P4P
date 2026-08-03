%% Piezo Voltage & Power vs Load Resistance
% Plots peak-to-peak voltage and power for single and dual piezo
% configurations against load resistor value, on a combined dual-axis plot.

clear; clc; close all;

%% --- Data ---
R = [97000, 47000, 9600, 4600];               % Resistor values (Ohms)

V_single = [53.7, 46.15, 23, 13.8];           % Vpp - Single Piezo (V)
V_dual   = [19.9, 11.6, 2.9, 1.42];           % Vpp - Dual Piezo (V)

P_single = [21.02140983, 32.04280707, 38.96452992, 29.27422074]; % mW
P_dual   = [2.886818107, 2.02443167, 0.619455003, 0.30995872];   % mW

%% --- Sort by resistor value (ascending) for cleaner line plotting ---
[R, sortIdx] = sort(R);
V_single = V_single(sortIdx);
V_dual   = V_dual(sortIdx);
P_single = P_single(sortIdx);
P_dual   = P_dual(sortIdx);

%% --- Combined dual-axis plot ---
figure('Color', 'w', 'Position', [100 100 900 600]);

yyaxis left
h1 = semilogx(R, V_single, '-o', 'LineWidth', 1.8, 'MarkerSize', 7, ...
    'DisplayName', 'Voltage - Single Piezo');
hold on
h2 = semilogx(R, V_dual, '--o', 'LineWidth', 1.8, 'MarkerSize', 7, ...
    'DisplayName', 'Voltage - Dual Piezo');
ylabel('Peak-to-Peak Voltage (V)');

yyaxis right
h3 = semilogx(R, P_single, '-s', 'LineWidth', 1.8, 'MarkerSize', 7, ...
    'DisplayName', 'Power - Single Piezo');
h4 = semilogx(R, P_dual, '--s', 'LineWidth', 1.8, 'MarkerSize', 7, ...
    'DisplayName', 'Power - Dual Piezo');
ylabel('Power (mW)');

xlabel('Load Resistance (\Omega)');
title('Piezo Output Voltage and Power vs Load Resistance');
legend([h1 h2 h3 h4], 'Location', 'best');
grid on;
set(gca, 'FontSize', 11);

hold off;

%% --- Force white background (figure + axes) ---
set(gcf, 'Color', 'k');
set(gca, 'Color', 'w');

%% --- Export as a white-background PNG (optional) ---
exportgraphics(gcf, 'piezo_voltage_power_vs_resistance.png', ...
    'BackgroundColor', 'white', 'Resolution', 300);