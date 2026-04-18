clear; clc; close all;

%% ================================
% DATA INPUT
% ================================
data = [
    0.1   141.871
    0.25  141.893
    0.5   141.933
    0.75  141.936
    1     142.023
    1.25  142.044
    1.5   142.149
    1.75  142.090
    2     142.149
    2.25  142.024
    2.5   142.216
    2.75  142.687
    3     142.867
    3.5   142.775
    4     142.898
    4.5   142.954
    5     142.800
    6     142.748
    7     142.842
    8     143.474
    9     143.170
    10    143.264
];

ds = data(:,1);
lap_time = data(:,2);

%% ================================
% SORT
% ================================
[ds, idx] = sort(ds);
lap_time = lap_time(idx);

%% ================================
% CONVERGED VALUE
% ================================
t_inf = mean(lap_time(ds <= 0.5));

%% ================================
% RELATIVE ERROR
% ================================
error_rel = abs(lap_time - t_inf) / t_inf;

%% ================================
% SMOOTH CURVE (visual only)
% ================================
ds_fit = linspace(min(ds), max(ds), 300);
lap_fit = interp1(ds, lap_time, ds_fit, 'pchip');
err_fit = interp1(ds, error_rel, ds_fit, 'pchip');

%% ================================
% COMMON STYLE
% ================================
color_main = [0.0000 0.4470 0.7410];

%% =========================================================
% FIGURE 1 — LAP TIME
% =========================================================
fig1 = figure('Color','w','Position',[100 100 900 550]);
ax1 = axes(fig1); hold(ax1,'on'); box(ax1,'on');

ax1.Color = 'w';
ax1.FontName = 'Helvetica';
ax1.FontSize = 11;
ax1.LineWidth = 1;
ax1.XColor = 'k';
ax1.YColor = 'k';
ax1.XLim   = [0 10];
ax1.XDir = 'reverse';
% ax1.YLim = [141.5 144];

grid off

scatter(ax1, ds, lap_time, 40, ...
    'o', ...
    'MarkerFaceColor', color_main, ...
    'MarkerEdgeColor', [0.1 0.1 0.1], ...
    'LineWidth', 0.5, ...
    'MarkerFaceAlpha', 0.9);

plot(ax1, ds_fit, lap_fit, ...
    'k-', 'LineWidth', 1.2);

yline(ax1, t_inf, '--', ...
    'Color', [0.2 0.2 0.2], ...
    'LineWidth', 1.2);

xlabel(ax1, 'ds [m]', 'FontName','Times','FontSize',12.5,'Color','k');
ylabel(ax1, 'Lap time [s]', 'FontName','Times','FontSize',12.5,'Color','k');

leg1 = legend(ax1, {'Simulation data','Trend','Converged value'}, ...
    'Location','northeast','Box','off','TextColor','k','FontSize',10);
set(leg1,'Color','none');

exportgraphics(fig1, 'mesh_convergence_laptime.pdf', ...
    'ContentType','vector','BackgroundColor','white');

%% =========================================================
% FIGURE 2 — RELATIVE ERROR
% =========================================================
fig2 = figure('Color','w','Position',[100 100 900 550]);
ax2 = axes(fig2); hold(ax2,'on'); box(ax2,'on');

ax2.Color = 'w';
ax2.FontName = 'Helvetica';
ax2.FontSize = 11;
ax2.LineWidth = 1;
ax2.XColor = 'k';
ax2.YColor = 'k';
ax2.XDir = 'reverse';
ax2.YScale = 'log';

grid off

scatter(ax2, ds, error_rel, 40, ...
    'o', ...
    'MarkerFaceColor', color_main, ...
    'MarkerEdgeColor', [0.1 0.1 0.1], ...
    'LineWidth', 0.5, ...
    'MarkerFaceAlpha', 0.9);

plot(ax2, ds_fit, err_fit, ...
    'k-', 'LineWidth', 1.2);

xlabel(ax2, 'ds [m]', 'FontName','Times','FontSize',12.5,'Color','k');
ylabel(ax2, 'Relative error [-]', 'FontName','Times','FontSize',12.5,'Color','k');

leg2 = legend(ax2, {'Relative error','Trend'}, ...
    'Location','northeast','Box','off','TextColor','k','FontSize',10);
set(leg2,'Color','none');

exportgraphics(fig2, 'mesh_convergence_error.pdf', ...
    'ContentType','vector','BackgroundColor','white');