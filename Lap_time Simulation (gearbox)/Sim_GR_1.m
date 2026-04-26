clear; clc; close all;

%% =========================================================
% LOAD DATA
% =========================================================
load('data.mat');
load('Theta.mat');
load('radius.mat');

%% =========================================================
% TRACK DISCRETIZATION STEP
% =========================================================
ds = .5;   % [m]

%% =========================================================
% TRACK PARAMETRIZATION
% =========================================================

[xpos, R, theta] = distance(M.LATITUDE, M.LONGITUDE, teste.distance, ...
    teste.rad, thetadata.xPosition_m_, thetadata.Theta_rad_, ds);

s = xpos(:);
R = R(:);
theta = theta(:);

N = length(s);

ds_vec = zeros(N,1);
ds_vec(1:N-1) = diff(s);
ds_vec(N) = ds;

%% =========================================================
% CHECKS ON TRACK
% =========================================================
if any(R == 0)
    error('R contains zero values.');
end

if any(~isfinite(theta))
    error('theta contains non-finite values.');
end

%% =========================================================
% VEHICLE PARAMETERS
% =========================================================
g       = 9.81;
rho     = 1.164;
m       = 220;
wb      = 1.3;
cgx     = 0.676;
cgh     = 0.600;
r_wheel = 0.3;
frontA  = 0.4;
Cd      = 0.5;
mu      = 0.95;
fW      = 0.015;
eta     = 1;
maxrpm  = 7000;

%% =========================================================
% POWER CURVE
% =========================================================

load('PCespanhol.mat');
t  = PC.Torque_Nm_(:);
rt = PC.MotorSpeed_rpm_(:);

% load('PCdatasheet.mat');         
% t  = PCdatasheet.Torque1(:);       %Por em comentário para testar outra
% rt = PCdatasheet.Speed1(:);                   %curva

% load('PCtestagem.mat');         
% t  = PCtestagem.Torque(:);
% rt = PCtestagem.Speed(:);


[rt, idx_sort] = sort(rt);
t = t(idx_sort);

[rt, idx_unique] = unique(rt, 'stable');
t = t(idx_unique);

if any(diff(rt) <= 0)
    error('RPM map is not strictly increasing.');
end

%% =========================================================
% LOCAL SPEED LIMIT FROM CURVATURE
% =========================================================
v_lim_curve = sqrt(mu * g .* cos(theta) .* abs(R)); % Suposição grosseira inicial (despreza lean angle e transferência de carga)
v_lim_curve(~isfinite(v_lim_curve)) = 300/3.6;
v_lim_curve(v_lim_curve > 300/3.6) = 300/3.6;

if any(~isreal(v_lim_curve))
    error('Complex values appeared in v_lim_curve. Check cos(theta) or R.');
end

%% =========================================================
% GEAR RATIO SWEEP
% =========================================================
precision = 100;
gear_ratio_vec = linspace(3.0, 4.0, precision);
lap_time_vec   = inf(size(gear_ratio_vec));

best_lap_time   = inf;
best_gear_ratio = NaN;
best_v          = [];

%Monitorização de Progresso
h = waitbar(0, 'Iniciando processamento...'); % Inicializa a barra
barcount = 0;

%% =========================================================
% LOOP OVER GEAR RATIO
% =========================================================
for k = 1:length(gear_ratio_vec)

    i_total = gear_ratio_vec(k);
    v = v_lim_curve;
    v_redline = maxrpm*(2*pi*r_wheel)/(60*i_total);
    
    %% =================================================
    % BACKWARD PASS CIRCULAR
    %% =================================================
    for ii = 0:N-1
        i = N - ii;
        next = i + 1;
        if next > N
            next = 1;
        end

        ds_local = ds_vec(i);

        % se Flat>Fmax Flat=Fmax
        F_max = mu * m * g * cos(theta(next));
        F_lat = min(m * v(next)^2 / abs(R(next)), F_max);

        F_long_tyre = sqrt(F_max^2 - F_lat^2);

        F_drag = 0.5 * rho * frontA * v(next)^2 * Cd;
        F_rr   = fW * m * g * cos(theta(next));
        F_slope = m * g * sin(theta(next));

        a_brake_avail = max((F_long_tyre + F_drag + F_rr + F_slope) / m, 0);
        a_stoppie     = abs(g * (cos(theta(next)) * (wb - cgx) / cgh ...
            + sin(theta(next)) ) + F_drag / m + F_rr/m );

        a_brake = min(a_brake_avail, a_stoppie);

        if a_brake < 0
            error('Warning: backward | negative a_brake at i = %d\n', i);
        end

        v_allowed = sqrt(v(next)^2 + 2*a_brake*ds_local);

        v(i) = min(v(i), v_allowed);
        v(i) = min(v(i), v_lim_curve(i));
        v(i) = min(v(i), v_redline);
    end

    %% =================================================
    % FORWARD PASS CIRCULAR
    %% =================================================
    for i = 1:N
        next = i + 1;
        if next > N
            next = 1;
        end

        ds_local = ds_vec(i);

        F_max = mu * m * g * cos(theta(i)) * (wb-cgx)/wb;
        F_lat = min((wb-cgx)/wb * m * v(i)^2 / abs(R(i)),F_max);

        F_long_tyre = sqrt(F_max^2 - F_lat^2);

        rpm = v(i) * i_total * 60 / (r_wheel * 2*pi);
        
        if rpm > maxrpm, torquemotor = 0;
        else, torquemotor = interp1(rt, t, rpm, 'pchip', 0); end

        F_motor = torquemotor * i_total * eta / r_wheel;

        F_drag = 0.5 * rho * frontA * v(i)^2 * Cd;
        F_rr   = fW * m * g * cos(theta(i));
        F_slope = m * g * sin(theta(i));

        F_drive = min(F_motor, F_long_tyre);

        a_drive = (F_drive - F_drag - F_rr - F_slope) / m;
        a_wheelie = g * ((cgx/cgh)*cos(theta(i)) - sin(theta(i))) ...
                    - F_drag/m - F_rr/m;

        a_acc = min(a_drive, a_wheelie);

        v_allowed = sqrt(v(i)^2 + 2*a_acc*ds_local);

        v(next) = min(v(next), v_allowed);
        v(next) = min(v(next), v_lim_curve(next));
        v(next) = min(v(next), v_redline);
    end
    
    nIter = 25;
    tol   = 1e-4;

    for it = 1:nIter
        v_old = v;
        if it == nIter, error('Simulation did not converge.'); end

        %% =================================================
        % BACKWARD PASS CIRCULAR
        %% =================================================
        for ii = 0:N-1
            i = N - ii;
            next = i + 1;
            if next > N
                next = 1;
            end

            ds_local = ds_vec(i);
            
            % se Flat>Fmax Flat=Fmax
            F_max = mu * m * g * cos(theta(next)); % Modelo de travagem completo com 2 rodas
            F_lat = min( m * v(next)^2 / abs(R(next)), F_max);

            F_long_tyre = sqrt(F_max^2 - F_lat^2);
    
            F_drag = 0.5 * rho * frontA * v(next)^2 * Cd;
            F_rr   = fW * m * g * cos(theta(next));
            F_slope = m * g * sin(theta(next));

            a_brake_avail = max((F_long_tyre + F_drag + F_rr + F_slope) / m, 0);
            a_stoppie     = abs(g * (cos(theta(next))* (wb - cgx) / cgh ...
                + sin(theta(next))) + F_drag / m + F_rr/m );

            a_brake = min(a_brake_avail, a_stoppie);

            if a_brake < 0
                error('Warning: backward | negative a_brake at i = %d\n', i);
            end

            v_allowed = sqrt(v(next)^2 + 2*a_brake*ds_local);

            v(i) = min(v(i), v_allowed);
            v(i) = min(v(i), v_lim_curve(i));
            v(i) = min(v(i), v_redline);
        end

        %% =================================================
        % FORWARD PASS CIRCULAR
        %% =================================================
        for i = 1:N
            next = i + 1;
            if next > N
                next = 1;
            end

            ds_local = ds_vec(i);
            a_local = (v_old(next)^2 - v_old(i)^2) / (2*ds_local) ;

            %Se F_lat>Fmax -> F_lat = F_max
            F_max = max(mu *( m * g * cos(theta(i)) * (wb - cgx)/wb + ...
                m * a_local * cgh/wb), 0);
            F_lat = min((wb-cgx)/wb * m * v(i)^2 / abs(R(i)), F_max);

            F_long_tyre = sqrt(F_max^2 - F_lat^2);

            rpm = v(i) * i_total * 60 / (r_wheel * 2*pi);
            
            if rpm > maxrpm, torquemotor = 0;
            else, torquemotor = interp1(rt, t, rpm, 'pchip', 0); end

            F_motor = torquemotor * i_total * eta / r_wheel;

            F_drag = 0.5 * rho * frontA * v(i)^2 * Cd;
            F_rr   = fW * m * g * cos(theta(i));
            F_slope = m * g * sin(theta(i));

            F_drive = min(F_motor, F_long_tyre);

            a_drive = (F_drive - F_drag - F_rr - F_slope) / m;
            a_wheelie = g * ((cgx/cgh)*cos(theta(i)) - sin(theta(i))) ...
                        - F_drag/m - F_rr/m;

            a_acc = min(a_drive, a_wheelie);

            v_allowed = sqrt(v(i)^2 + 2*a_acc*ds_local);

            v(next) = min(v(next), v_allowed);
            v(next) = min(v(next), v_lim_curve(next));
            v(next) = min(v(next), v_redline);
        end

        if max(abs(v - v_old)) < tol, break; end

    end

    %% =================================================
    % LAP TIME
    %% =================================================
    dt = zeros(N,1);

    for i = 1:N
        next = i + 1;
        if next > N
            next = 1;
        end

        ds_local = ds_vec(i);
        v_avg = 0.5 * (v(i) + v(next));

        if v_avg <= 0
            error('Warning: non-positive average speed at segment %d\n', i);
        else
            dt(i) = ds_local / v_avg;
        end
    end

    lap_time = sum(dt);
    lap_time_vec(k) = lap_time;

    if lap_time < best_lap_time
        best_lap_time   = lap_time;
        best_gear_ratio = i_total;
        best_v          = v;
        best_dt = dt;
    end

    barcount = barcount + 1;
    perc = barcount / precision;
    waitbar(perc, h, sprintf('Progresso: %d%%', round(perc*100)));
end

%% =========================================================
% POST-PROCESS BEST SOLUTION
% =========================================================
v_best = best_v;
phi_best     = atan( v_best.^2 ./ (R * g) );
best_v_lim = v_lim_curve;
ax_best = zeros(N,1);
ay_best = zeros(N,1);

F_lat_best   = zeros(N,1);
F_drag_best  = zeros(N,1);
F_rr_best    = zeros(N,1);
F_slope_best = zeros(N,1);
S_best = zeros(N,1);
F_max_best   = zeros(N,1);
E_seg        = zeros(N,1);

%% =========================================================
% ANÁLISE DE ENERGIA E BATERIA
% =========================================================
n_laps       = 6;       % número de voltas da prova
safety_coeff = 1.25;     % margem de segurança (25%)
usable_fraction = 0.85;
eta_regen    = 0.0;      % eficiência de regeneração (0 = sem regen)

for i = 1:N
    next = i + 1;
    if next > N
        next = 1;
    end

    ds_local = ds_vec(i);
    
    % Acelerações
    ax_best(i) = (v_best(next)^2 - v_best(i)^2) / (2*ds_local);
    ay_best(i) = v_best(i)^2 / R(i);

    % Forças disponíveis no pneu
    if ax_best(i) < 0
        F_max = mu * m * g * cos(theta(i));
        F_lat = min(m * v_best(i)^2 /abs(R(i)), F_max);
    else
        F_max = max(mu * (m * g * cos(theta(i)) * (wb - cgx)/wb + ...
                m * ax_best(i) * cgh/wb), 0);
        F_lat = min((wb-cgx)/wb * m * v_best(i)^2 / abs(R(i)), F_max);
    end
    F_max_best(i) = F_max;
    F_lat_best(i) = F_lat;

    F_long_tyre = sqrt(F_max^2 - F_lat^2);

    % RPM / motor
    rpm = v_best(i) * best_gear_ratio * 60 / (r_wheel * 2*pi);

    if rpm > maxrpm, torquemotor = 0;
    else, torquemotor = interp1(rt, t, rpm, 'pchip', 0); end

    F_motor = torquemotor * best_gear_ratio * eta / r_wheel;

    % Resistências
    F_drag_best(i) = 0.5 * rho * frontA * v_best(i)^2 * Cd;
    F_rr_best(i)   = fW * m * g * cos(theta(i));
    F_slope_best(i)= m * g * sin(theta(i));

    % Força motriz efetiva
    if ax_best(i) < 0
        S_best(i) = 0;
    else
        S_best(i) = min(F_motor, F_long_tyre);
    end

    if S_best(i) > 0
        % Energia mecânica na roda → energia eléctrica (revertendo eta)
        E_seg(i) = S_best(i) * ds_local;
    else
        % Travagem: recuperação opcional -> Não está bem modelado
        E_seg(i) = -abs(S_best(i)) * ds_local * eta_regen;
    end

end
close(h); % Fecha a barra ao terminar

best_ax = ax_best / g;
best_ay = ay_best / g;

%% =========================================================
% RESULTS
% =========================================================
fprintf('\nMECHANICAL EVALUATION\n');
fprintf('-----------------------------------------------------------------------\n');
fprintf('Best Gear Ratio   = %.4f\n', best_gear_ratio);
fprintf('Minimum Lap-time  = %.3f s\n', best_lap_time);
fprintf('Maximum Velocity  = %.2f km/h\n', max(v_best)*3.6);
fprintf('=======================================================================\n');
% --- Energia por volta ---
E_lap_J   = sum(E_seg);           % [J]
E_lap_Wh  = E_lap_J / 3600;       % [Wh]
E_lap_kWh = E_lap_Wh / 1000;      % [kWh]

% --- Energia total da prova ---
E_total_kWh  = E_lap_kWh * n_laps;
E_battery_kWh = E_total_kWh * safety_coeff / usable_fraction;

% --- Potência média e de pico ---
P_inst = S_best .* v_best;           % [W] — potência instantânea na roda
P_peak_kW = max(P_inst) / 1000;
P_avg_kW  = (E_lap_J / best_lap_time) / 1000;

% --- Output ---
fprintf('ENERGETIC EVALUATION\n');
fprintf('-----------------------------------------------------------------------\n');
fprintf('Energy Spent per lap:    %.3f kWh\n', E_lap_kWh);
fprintf('Total Energy Spent (%.1f laps):        %.3f kWh\n', n_laps, E_total_kWh);
fprintf('Battery Capacity (w/ %.0f%% margin): %.3f kWh\n', ((safety_coeff-1)/usable_fraction)*100, E_battery_kWh);
fprintf('-----------------------------------------------------------------------\n');
fprintf('Peak Power (wheel):          %.2f kW\n', P_peak_kW);
fprintf('Average Power:         %.2f kW\n', P_avg_kW);
fprintf('=======================================================================\n');


%% =========================================================
% PLOTS
% =========================================================

set(groot,'defaultLegendTextColor','k') % FIX GLOBAL

colors = lines(5);

output_folder = 'direct_drive_plots';
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

%% speed
fig = figure('Name', 'Velocity and Forces Profile', 'Color', 'w');

subplot(2,1,1)
ax1 = gca; hold(ax1,'on'); box(ax1,'on');
set(ax1,'Color','w','FontName','Helvetica','FontSize',25,...
    'LineWidth',1,'XColor','k','YColor','k');

plot(s, best_v_lim*3.6, '--', 'LineWidth', 0.8, 'Color', [0.2 0.5 0.9]);
plot(s, v_best*3.6, 'LineWidth', 1.2, 'Color', [0.95 0.2 0.1]);

grid(ax1,'on'); ax1.XGrid='off';
ax1.GridColor=[0.82 0.82 0.82]; ax1.GridAlpha=0.35;

xlabel('s [m]','FontSize',30,'FontName','Times');
ylabel('v [km/h]','FontSize',30,'FontName','Times');

leg1 = legend('v_{lim,curve}','v_{final}','Location','northeast','Box','off');
leg1.FontSize = 25;
leg1.FontName = 'Helvetica';
set(findall(leg1,'Type','text'),'Color',[0 0 0]);

subplot(2,1,2)
ax2 = gca; hold(ax2,'on'); box(ax2,'on');
set(ax2,'Color','w','FontName','Helvetica','FontSize',25,...
    'LineWidth',1,'XColor','k','YColor','k');

plot(s, S_best,        'LineWidth',1.1,'Color',[0.1 0.7 0.2]);   % drive
plot(s, F_drag_best,   'LineWidth',1.0,'Color',[0.0 0.45 0.75]); % drag
plot(s, F_rr_best,     'LineWidth',1.0,'Color',[0.75 0.2 0.75]); % rr
plot(s, F_slope_best,  'LineWidth',1.0,'Color',[0.9 0.6 0.1]);   % slope

grid(ax2,'on'); ax2.XGrid='off';
ax2.GridColor=[0.82 0.82 0.82]; ax2.GridAlpha=0.35;

xlabel('s [m]','FontSize',30,'FontName','Times');
ylabel('Force [N]','FontSize',30,'FontName','Times');

leg2 = legend('F_{drive}','F_{drag}','F_{rr}','F_{slope}','Location','northeast','Box','off');
leg2.FontSize = 25;
leg2.FontName = 'Helvetica';
set(findall(leg2,'Type','text'),'Color',[0 0 0]);

exportgraphics(fig, fullfile(output_folder,'speed_forces.pdf'),...
    'ContentType','vector','BackgroundColor','white');

%% Power Consumption

t_acumulado = [0; cumsum(best_dt(1:end-1))];
P_seg = E_seg ./ best_dt;

fig = figure('Name','Power Profile','Color','w');
ax = axes(fig); hold(ax,'on'); box(ax,'on');

set(ax,'Color','w','FontName','Helvetica','FontSize',25,...
    'LineWidth',1,'XColor','k','YColor','k');

area(t_acumulado, P_seg*1e-3,...
    'FaceColor',[0.9 0.4 0.1],...
    'FaceAlpha',0.35,...
    'EdgeColor',[0.6 0.2 0.05],...
    'LineWidth',0.8);

grid(ax,'on'); ax.XGrid='off';
ax.YLim = [0 max(P_seg * 1e-3)*1.1];
ax.GridColor=[0.82 0.82 0.82]; ax.GridAlpha=0.35;

xlabel('Lap time [s]','FontSize',30,'FontName','Times');
ylabel('Power [kW]','FontSize',30,'FontName','Times');

leg = legend('Segment Power','Location','northeast','Box','off');
leg.FontSize = 25;
leg.FontName = 'Helvetica';
leg.TextColor = [0 0 0];
leg.EdgeColor = 'none';
set(findall(leg,'Type','text'),'Color',[0 0 0]);

exportgraphics(fig, fullfile(output_folder,'power_profile.pdf'),...
    'ContentType','vector','BackgroundColor','white');

%% Long vs Lat acceleration

fig = figure('Name','Acceleration','Color','w');
ax = axes(fig); hold(ax,'on');

set(ax,'Color','w','FontName','Helvetica','FontSize',25,...
    'LineWidth',1,'XColor','k','YColor','k');

box(ax,'off')

plot(s, best_ax,'LineWidth',1.2,'Color',[0.85 0.1 0.1]);   % a_x (longitudinal)
plot(s, best_ay,'LineWidth',1.2,'Color',[0.1 0.35 0.85]);  % a_y (lateral)

ymax = max(abs([best_ax; best_ay]));
ylim(ax, 1.05*[-ymax ymax]);

grid(ax,'on'); ax.XGrid='off';
ax.GridColor=[0.82 0.82 0.82]; ax.GridAlpha=0.30;

xlabel('s [m]','FontSize',30,'FontName','Times');
ylabel('a [g]','FontSize',30,'FontName','Times');

leg = legend('a_x','a_y','Location','northeast','Box','off');
leg.FontSize = 25;
leg.FontName = 'Helvetica';
leg.TextColor = [0 0 0];
leg.EdgeColor = 'none';
set(findall(leg,'Type','text'),'Color',[0 0 0]);

exportgraphics(fig, fullfile(output_folder,'latlon_acceleration.pdf'),...
    'ContentType','vector','BackgroundColor','white');

%% Gear Ratio vs Lap-Time

fig = figure('Name','Gear Ratio vs Lap Time','Color','w');
ax = axes(fig); hold(ax,'on'); box(ax,'on');
pbaspect([1 1 1])

set(ax,'Color','w','FontName','Helvetica','FontSize',25,...
    'LineWidth',1,'XColor','k','YColor','k');

scatter(gear_ratio_vec, lap_time_vec, 35, lap_time_vec, 'filled');
colormap(turbo)

plot(gear_ratio_vec, lap_time_vec, 'k','LineWidth',0.8);

[min_lap, idx_opt] = min(lap_time_vec);
opt_ratio = gear_ratio_vec(idx_opt);

plot(opt_ratio, min_lap,...
    'ko','MarkerSize',8,'MarkerFaceColor','w','LineWidth',1);

cb = colorbar;
cb.Color = 'k';
cb.FontName = 'Helvetica';
cb.FontSize = 25;
cb.Label.String = 'Lap Time [s]';
cb.Label.FontName = 'Times';
cb.Label.FontSize = 30;
cb.Label.Color = 'k';

clim([141.5 143.5])
ylim([141.5 143.5])

% eixo x simplificado
xlim([3 4])
xticks([3 3.5 4])

grid(ax,'on'); ax.XGrid='off';
ax.GridColor=[0.82 0.82 0.82]; ax.GridAlpha=0.35;

xlabel('i_f','FontSize',30,'FontName','Times');
ylabel('');

exportgraphics(fig, fullfile(output_folder,'gr_single.pdf'),...
    'ContentType','vector','BackgroundColor','white');

%% =========================================================
% G-G-V Projection
% =========================================================

fig = figure('Name','G-G-V Projection','Color','w');
ax = axes(fig); hold(ax,'on');

set(ax,'Color','w','FontName','Helvetica','FontSize',25,...
    'LineWidth',1,'XColor','k','YColor','k');

ay_lin = linspace(min(best_ay), max(best_ay), 220);
ax_lin = linspace(min(best_ax), max(best_ax), 220);

[AY, AX] = meshgrid(ay_lin, ax_lin);

F = scatteredInterpolant(best_ay, best_ax, v_best,...
    'natural','nearest');

V = F(AY, AX);

V = smoothdata(V,1,'gaussian',7);
V = smoothdata(V,2,'gaussian',7);
k = convhull(best_ay, best_ax);
in = inpolygon(AY, AX, best_ay(k), best_ax(k));
V(~in) = NaN;

contourf(AY, AX, V, 40, 'LineStyle','none');
colormap(turbo)

contour(AY, AX, V, 16,...
    'LineColor',[0.2 0.2 0.2],...
    'LineWidth',0.5);

clim([min(v_best) max(v_best)])

cb = colorbar;
cb.Color = 'k';
cb.FontName = 'Helvetica';
cb.FontSize = 25;
cb.Label.String = 'Velocity [m/s]';
cb.Label.FontName = 'Times';
cb.Label.FontSize = 30;

xlabel('a_y [g]','FontSize',30,'FontName','Times');
ylabel('a_x [g]','FontSize',30,'FontName','Times');

axis equal
grid on

ax.GridColor = [0.82 0.82 0.82];
ax.GridAlpha = 0.30;

axis tight

ay_min = min(best_ay);
ay_max = max(best_ay);
ax_min = min(best_ax);
ax_max = max(best_ax);

xlim([-1.0 1.0]);
ylim([-1.2 0.5]);

exportgraphics(fig, fullfile(output_folder,'ggv_single.pdf'),...
    'ContentType','vector','BackgroundColor','white');

%% =========================================================
% LOCAL FUNCTION: distance
% =========================================================
function [xpos, R, theta] = distance(lat, lon, xr, r, pos, rad, ds)

    rt_earth = 6371000;

    lat = lat(:);
    lon = lon(:);
    xr  = xr(:);
    r   = r(:);
    pos = pos(:);
    rad = rad(:);

    latr = deg2rad(lat);
    lonr = deg2rad(lon);

    N = numel(latr);
    d = zeros(N,1);

    for i = 2:N
        dlat = latr(i) - latr(i-1);
        dlon = lonr(i) - lonr(i-1);

        a = sin(dlat/2)^2 + cos(latr(i-1))*cos(latr(i))*sin(dlon/2)^2;
        c = 2 * asin(sqrt(a));
        d(i) = d(i-1) + rt_earth*c;
    end

    xpos = (0:ds:floor(max(d)))';
    % if xpos(end) < max(d)
    %     xpos = [xpos; max(d)];
    % end

    r_straight_thr = 1e6;
    r(abs(r) < .1) = inf;
    kappa_raw = zeros(size(r));

    idx_curve = isfinite(r) & abs(r) < r_straight_thr;
    kappa_raw(idx_curve) = 1 ./ r(idx_curve);
    kappa_raw(~isfinite(kappa_raw)) = 0;

    valid_xr = isfinite(xr) & isfinite(kappa_raw);
    xr_k = xr(valid_xr);
    kappa_k = kappa_raw(valid_xr);

    [xr_k, idx_sort] = sort(xr_k);
    kappa_k = kappa_k(idx_sort);

    [xr_k, idx_unique] = unique(xr_k, 'stable');
    kappa_k = kappa_k(idx_unique);

    valid_pos = isfinite(pos) & isfinite(rad);
    pos_t = pos(valid_pos);
    rad_t = rad(valid_pos);

    [pos_t, idx_sort_t] = sort(pos_t);
    rad_t = rad_t(idx_sort_t);

    [pos_t, idx_unique_t] = unique(pos_t, 'stable');
    rad_t = rad_t(idx_unique_t);

    % puro: sem smooth
    kappa = interp1(xr_k, kappa_k, xpos, 'pchip', 'extrap');
    theta = interp1(pos_t, rad_t, xpos, 'pchip', 'extrap');
    
    kappa = smoothdata(kappa, 'movmean', 5/ds);
    theta = smoothdata(theta, 'movmean', 5/ds);
       
    eps_k = 1e-9;
    R = inf(size(kappa));
    idx_nonzero = abs(kappa) > eps_k;
    R(idx_nonzero) = 1 ./ kappa(idx_nonzero);

    if numel(xpos) ~= numel(R) || numel(R) ~= numel(theta)
        error('Error in Track Parametrization: Different Dimensions');
    end

    disp(' - Track Successfully Parametrized');
end