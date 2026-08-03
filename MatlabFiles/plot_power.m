% =========================================================
%  BLE Power Consumption - Oscilloscope Data Plotter
%  Resistor: 10.6 ohms
%  Run: plot_power
% =========================================================

R = 10.6;  % shunt resistor (ohms)

% ---------------------------------------------------------
%  Load raw data
%  Columns paired as: [t1 v1  t2 v2 ... t8 v8]
% ---------------------------------------------------------
raw = readmatrix('Plot_Matlab.xlsx', 'NumHeaderLines', 1);

% Extract each capture, strip NaN rows independently per pair
[t_poll,  i_poll]  = getCapture(raw, 1,  2,  R);   % Case 1 Polling
[t_del,   i_del]   = getCapture(raw, 4,  5,  R);   % Case 1 Delay
[t_sus,   i_sus]   = getCapture(raw, 7,  8,  R);   % Case 1 Suspend

[t_tx3,   i_tx3]   = getCapture(raw, 10,  11,  R);   % Case 2  TX=3ms
[t_tx100, i_tx100] = getCapture(raw, 13,  14, R);   % Case 2  TX=100ms
[t_tx1000,i_tx1000]= getCapture(raw, 16, 17, R);   % Case 2  TX=1000ms

[t_base,  i_base]  = getCapture(raw, 19, 20, R);   % Case 3 Baseline max
[t_near,  i_near]  = getCapture(raw, 22, 23, R);   % Case 3 Near minimum

% =========================================================
%  FIGURE 1  Sleep mode comparison (overlaid)
%  25 Hz, 100 ms TX, LPEN off
% =========================================================
figure('Name','Case 1 - Sleep Mode Comparison','NumberTitle','off', ...
       'Position',[80 200 900 400]);
hold on;
p1 = plot(t_poll, i_poll, 'Color',[0.85 0.33 0.10], 'LineWidth',1.2);
p2 = plot(t_del,  i_del,  'Color',[0.00 0.45 0.74], 'LineWidth',1.2);
p3 = plot(t_sus,  i_sus,  'Color',[0.47 0.67 0.19], 'LineWidth',1.2);
hold off;
xlabel('Time (ms)');
ylabel('Current (mA)');
title('Sleep Mode Comparison  --  25 Hz, 100 ms TX, LPEN off');
legend([p1 p2 p3], 'Polling','Delay','Suspend', 'Location','best');
grid on;
set(gca,'FontSize',11);

% =========================================================
%  FIGURE 2  TX interval impact (overlaid)
%  Delay mode, 25 Hz, LPEN off
% =========================================================
figure('Name','Case 2 - TX Interval Impact','NumberTitle','off', ...
       'Position',[100 180 900 400]);
hold on;
p1 = plot(t_tx3,    i_tx3,    'Color',[0.85 0.33 0.10], 'LineWidth',1.2);
p2 = plot(t_tx100,  i_tx100,  'Color',[0.00 0.45 0.74], 'LineWidth',1.2);
p3 = plot(t_tx1000, i_tx1000, 'Color',[0.47 0.67 0.19], 'LineWidth',1.2);
hold off;
xlabel('Time (ms)');
ylabel('Current (mA)');
title('TX Interval Impact  --  Delay mode, 25 Hz, LPEN off');
legend([p1 p2 p3], 'TX = 3 ms','TX = 100 ms','TX = 1000 ms', 'Location','best');
grid on;
set(gca,'FontSize',11);

% =========================================================
%  FIGURE 3  Baseline max vs Near minimum (side by side)
% =========================================================
figure('Name','Case 3 - Suspend Mode Best vs Worse Case','NumberTitle','off', ...
       'Position',[120 160 1100 400]);

subplot(1,2,1);
plot(t_base, i_base, 'Color',[0.85 0.33 0.10], 'LineWidth',1.2);
xlabel('Time (ms)');
ylabel('Current (mA)');
title('Baseline Max  --  400 Hz, 3 ms TX');
grid on;
xlim([min(t_base) max(t_base)]);
set(gca,'FontSize',11);

subplot(1,2,2);
plot(t_near, i_near, 'Color',[0.00 0.45 0.74], 'LineWidth',1.2);
xlabel('Time (ms)');
ylabel('Current (mA)');
title('Near Minimum  --  1 Hz, 1000 ms TX');
grid on;
xlim([min(t_near) max(t_near)]);
set(gca,'FontSize',11);

sgtitle('Suspend Mode Best vs Worst Case Power Consumption', ...
        'FontSize',13,'FontWeight','bold');

% =========================================================
%  LOCAL FUNCTION  (must be at end of script file)
% =========================================================
function [t_ms, i_mA] = getCapture(raw, t_col, v_col, R)
    t    = raw(:, t_col);
    v    = raw(:, v_col);
    mask = ~isnan(t) & ~isnan(v);
    t_ms = t(mask) * 1000;         % s -> ms
    i_mA = (v(mask) / R) * 1000;  % V -> mA
end