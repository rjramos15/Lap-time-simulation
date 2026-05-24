clear; clc; close all

%% =========================================================
% POWER CURVE
% =========================================================

load('PCespanhol.mat');
torque  = PC.Torque_Nm_(:);
rpm = PC.MotorSpeed_rpm_(:);


output_folder = 'extra_plots';
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

%% =========================================================
% MOTOR TORQUE CURVE
% =========================================================

color_main = [0.0000 0.4470 0.7410];

fig = figure('Name','Motor Torque Curve','Color','w');
ax = axes(fig); hold(ax,'on'); box(ax,'on');

set(ax,...
    'Color','w',...
    'FontName','Times',...
    'FontSize',25,...
    'LineWidth',1,...
    'XColor','k',...
    'YColor','k');

%% Main curve: raw data, no smoothing, no data treatment
plot(rpm, torque,...
    'LineWidth',1.2,...
    'Color', [0.1 0.1 0.1]);

% %% Markers every 500 rpm only within the real curve range
% rpm_marks = ceil(min(rpm)/500)*500 : 500 : floor(max(rpm)/500)*500;
% 
% torque_marks = interp1(rpm, torque, rpm_marks, 'linear');
% 
% plot(rpm_marks, torque_marks,...
%     'o',...
%     'MarkerSize',5.5,...
%     'MarkerFaceColor',color_main,...
%     'MarkerEdgeColor',[0.1 0.1 0.1],...
%     'LineWidth',0.5);

%% Axes
grid on
% ax.XGrid = 'off';
ax.YGrid = 'off';
ax.GridColor = [0.86 0.86 0.86];
ax.GridAlpha = 0.22;

xlabel('Motor speed [rpm]',...
    'FontSize',30,...
    'FontName','Times');

ylabel('Torque [Nm]',...
    'FontSize',30,...
    'FontName','Times');

xlim([min(rpm) max(rpm)])
ylim([0 max(torque)*1.08])

xticks(0:1000:ceil(max(rpm)/1000)*1000)

exportgraphics(fig,...
    fullfile(output_folder,'motor_torque_curve.pdf'),...
    'ContentType','vector',...
    'BackgroundColor','white');