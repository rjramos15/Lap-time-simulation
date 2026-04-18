clear; clc; close all;

%% =========================================================
% DATA INPUT
% =========================================================
cases = {
 '1g_m220_e1',      141.933,  0.742,    1,      220,   1;
 '1g_m220_e.95',    142.478,  0.720,    1,      220,   0.95;
 '1g_m220_e.90',    143.107,  0.698,    1,      220,   0.90;

 '2g_m220_e1',      141.623,  0.763,    2,      220,   1;
 '2g_m220_e.95',    142.115,  0.741,    2,      220,   0.95;
 '2g_m220_e.90',    142.668,  0.718,    2,      220,   0.90;
 '2g_m225_e1',      141.743,  0.769,    2,      225,   1;
 '2g_m225_e.95',    142.246,  0.746,    2,      225,   0.95;
 '2g_m225_e.90',    142.814,  0.723,    2,      225,   0.90;

 '3g_m220_e1',      141.499,  0.767,    3,      220,   1;
 '3g_m220_e.95',    141.961,  0.746,    3,      220,   0.95;
 '3g_m220_e.90',    142.506,  0.723,    3,      220,   0.90;
 '3g_m225_e1',      141.601,  0.773,    3,      225,   1;
 '3g_m225_e.95',    142.084,  0.752,    3,      225,   0.95;
 '3g_m225_e.90',    142.649,  0.728,    3,      225,   0.90;
};

lap    = cell2mat(cases(:,2));
energy = cell2mat(cases(:,3));
gears  = cell2mat(cases(:,4));
mass   = cell2mat(cases(:,5));

%% =========================================================
% FIGURE STYLE
% =========================================================
fig = figure('Color','w','Position',[100 100 900 550]);
ax = axes(fig); hold(ax,'on'); box(ax,'on');

ax.Color = 'w';
ax.FontName = 'Helvetica';
ax.FontSize = 11;
ax.LineWidth = 1;
ax.XColor = 'k';
ax.YColor = 'k';

grid off

%% =========================================================
% STYLE DEFINITIONS
% =========================================================
gear_vals = unique(gears);
line_styles = {'-','-.',':'};   % 1, 2, 3 gears

color_points = [0.0000 0.4470 0.7410];
marker_size = 45;

%% =========================================================
% SCATTER (marker by mass)
% =========================================================
for i = 1:length(lap)

    if mass(i) == 220
        marker_type = 'o';
    else
        marker_type = '^';
    end

    scatter(ax, energy(i), lap(i), marker_size, ...
        marker_type, ...
        'MarkerFaceColor', color_points, ...
        'MarkerEdgeColor', [0.1 0.1 0.1], ...
        'LineWidth', 0.5, ...
        'MarkerFaceAlpha', 0.9, ...
        'HandleVisibility','off');
end

%% =========================================================
% REGRESSIONS + LABELS
% =========================================================
unique_pairs = unique([gears mass], 'rows');

for k = 1:size(unique_pairs,1)

    g = unique_pairs(k,1);
    m = unique_pairs(k,2);

    idx = (gears == g) & (mass == m);
    if sum(idx) < 2
        continue
    end

    x = energy(idx);
    y = lap(idx);

    % linear regression
    p = polyfit(x, y, 1);

    x_fit = linspace(min(x), max(x), 100);
    y_fit = polyval(p, x_fit);

    style = line_styles{find(gear_vals == g, 1)};

    plot(ax, x_fit, y_fit, ...
        'Color', 'k', ...
        'LineStyle', style, ...
        'LineWidth', 1.2, ...
        'HandleVisibility','off');
end

%% =========================================================
% LABELS
% =========================================================
xlabel(ax, 'Energy per lap [kWh]', 'Color','k','FontSize',12.5,'FontName','Times');
ylabel(ax, 'Lap time [s]',        'Color','k','FontSize',12.5,'FontName','Times');

%% =========================================================
% CUSTOM LEGEND
% =========================================================
% mass markers
h220 = scatter(ax, nan, nan, marker_size, 'o', ...
    'MarkerFaceColor', color_points, ...
    'MarkerEdgeColor', [0.1 0.1 0.1], ...
    'LineWidth', 0.5);

h225 = scatter(ax, nan, nan, marker_size, '^', ...
    'MarkerFaceColor', color_points, ...
    'MarkerEdgeColor', [0.1 0.1 0.1], ...
    'LineWidth', 0.5);

% gear line styles
h1 = plot(ax, nan, nan, 'k-',  'LineWidth', 1.2);
h2 = plot(ax, nan, nan, 'k-.', 'LineWidth', 1.2);
h3 = plot(ax, nan, nan, 'k:', 'LineWidth', 1.2);

leg = legend(ax, [h220 h225 h1 h2 h3], ...
    {'220 kg','225 kg','1 gear','2 gears','3 gears'}, ...
    'Location','northeast', ...
    'Box','off', ...
    'TextColor','k', ...
    'FontSize',10);

set(leg, 'Color', 'none');

%% =========================================================
% EXPORT
% =========================================================
exportgraphics(fig, 'lap_vs_energy.pdf', ...
    'ContentType','vector', ...
    'BackgroundColor','white');