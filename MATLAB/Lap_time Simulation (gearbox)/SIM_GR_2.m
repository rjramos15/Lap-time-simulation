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
m       = 225;
wb      = 1.3;
cgx     = 0.676;
cgh     = 0.600;
r_wheel = 0.3;
frontA  = 0.4;
Cd      = 0.5;
mu      = .95;
fW      = 0.015;
eta     = .95;
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
v_lim_curve = sqrt(mu * g .* cos(theta) .* abs(R));
v_lim_curve(~isfinite(v_lim_curve)) = 300/3.6;
v_lim_curve(v_lim_curve > 300/3.6) = 300/3.6;

if any(~isreal(v_lim_curve))
    error('Complex values appeared in v_lim_curve. Check cos(theta) or R.');
end

%% =========================================================
% GEAR RATIO SWEEP
% =========================================================
precision = 10;
gear_ratio_planetary_first = linspace(1.37, 1.38, precision);
gear_ratio_final = linspace(3.87, 3.88, precision);
i_current = zeros(N,1);

lap_time_vec   = inf( precision, precision);

best_lap_time   = inf;
best_ratio_vec = NaN;
best_v          = [];
best_v_lim      = [];

%Monitorização de Progresso
h = waitbar(0, 'Iniciando processamento...'); % Inicializa a barra
barcount = 0;

%% =========================================================
% LOOP OVER GEAR RATIO
% =========================================================

for kk = 1:precision
for k = 1:precision

    % i_total = gear_ratio_vec(k);
    i_first = gear_ratio_planetary_first(kk) * gear_ratio_final(k);
    i_second = gear_ratio_final(k);
    i_current(:) = i_second;

    v_redline = maxrpm*(2*pi*r_wheel)/(60*i_second);
    v = v_lim_curve;
    
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

        F_long_tyre = sqrt((F_max)^2 - F_lat^2);

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
        
        rpm_first = v(i) * i_first * 60 / (r_wheel * 2*pi);
        rpm_second = v(i) * i_second * 60 / (r_wheel *2*pi);
        
        if rpm_first  > maxrpm, torque_first  = 0;
        else, torque_first  = interp1(rt, t, rpm_first,  'pchip', 0); end
        
        if rpm_second > maxrpm, torque_second = 0;
        else, torque_second = interp1(rt, t, rpm_second, 'pchip', 0); end

        F_first = min(torque_first * i_first * eta / r_wheel, F_long_tyre);
        F_second = min(torque_second * i_second * eta / r_wheel, F_long_tyre);

        if (i_current(i) == i_first)
            if (F_second > F_first * 1.00 || rpm_first > maxrpm)
                i_current(i) = i_second;
                F_motor = F_second;
            else
                F_motor = F_first;
            end
        elseif (i_current(i) == i_second)
            if (F_first > F_second * 1.00 && rpm_first < maxrpm)
                i_current(i) = i_first;
                F_motor = F_first;
            else
                F_motor = F_second;
            end
        end

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
    tol   = 1e-2;
    
    for it = 1:nIter
        v_old = v;
        i_old = i_current;
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
            F_max = mu * m * g * cos(theta(next));
            F_lat = min(m * v(next)^2 / abs(R(next)), F_max);

            F_long_tyre = sqrt((F_max)^2 - F_lat^2);
    
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

            F_max = max(mu *( m * g * cos(theta(i)) * (wb - cgx)/wb + ...
                m * a_local * cgh/wb), 0);
            F_lat = min((wb-cgx)/wb * m * v(i)^2 / abs(R(i)), F_max);

            if F_lat > F_max
                error('Warning: forward | F_lat > F_max at i = %d\n', i);
            end

            F_long_tyre = sqrt(F_max^2 - F_lat^2);
            
            rpm_first = v(i) * i_first * 60 / (r_wheel * 2*pi);
            rpm_second = v(i) * i_second * 60 / (r_wheel *2*pi);
            
            if rpm_first  > maxrpm, torque_first  = 0;
            else, torque_first  = interp1(rt, t, rpm_first,  'pchip', 0); end
            
            if rpm_second > maxrpm, torque_second = 0;
            else, torque_second = interp1(rt, t, rpm_second, 'pchip', 0); end

            F_first = min(torque_first * i_first * eta / r_wheel, F_long_tyre);
            F_second = min(torque_second * i_second * eta / r_wheel, F_long_tyre);

            if (i_current(i) == i_first)
                if (F_second > F_first * 1.00 || rpm_first > maxrpm)
                    i_current(i) = i_second;
                    F_motor = F_second;
                % elseif rpm_first > maxrpm
                %     F_motor = 0;
                else
                    F_motor = F_first;
                end
            elseif (i_current(i) == i_second)
                if (F_first > F_second * 1.00 && rpm_first < maxrpm)
                    i_current(i) = i_first;
                    F_motor = F_first;
                % elseif rpm_second > maxrpm
                %     F_motor = 0;
                else
                    F_motor = F_second;
                end
            end

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

        if max(abs(v - v_old)) < tol && all(i_old == i_current)
            break;
        end
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
    lap_time_vec( kk, k) = lap_time;

    if lap_time < best_lap_time
        best_lap_time   = lap_time;
        %best_ratio_vec = i_total;
        best_v          = v;
        best_v_lim      = v_lim_curve;
        best_gear_planetary_first = gear_ratio_planetary_first(kk);
        best_gear_final = gear_ratio_final(k);
        best_ratio_vec = i_current;
        best_perc_1 = round(sum(best_ratio_vec == i_first) / N * 100, 2);
        best_perc_2 = round(sum(best_ratio_vec == i_second) / N * 100, 2);
        best_dt = dt;
    end

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

ax_best = zeros(N,1);
ay_best = zeros(N,1);

F_lat_best   = zeros(N,1);
F_drag_best  = zeros(N,1);
F_rr_best    = zeros(N,1);
F_slope_best = zeros(N,1);
S_best       = zeros(N,1);
F_max_best   = zeros(N,1);
E_seg        = zeros(N, 1);

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
    rpm = v_best(i) * best_ratio_vec(i) * 60 / (r_wheel * 2*pi);

    if rpm > maxrpm, torquemotor = 0;
    else, torquemotor = interp1(rt, t, rpm, 'pchip', 0); end

    F_motor = torquemotor * best_ratio_vec(i) * eta / r_wheel;

    % Resistências
    F_drag_best(i) = 0.5 * rho * frontA * v_best(i)^2 * Cd;
    F_rr_best(i)   = fW * m * g * cos(theta(i));
    F_slope_best(i) = m * g * sin(theta(i));

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
        % Travagem: recuperação opcional
        E_seg(i) = -abs(S_best(i)) * ds_local * eta_regen;
    end

end
close(h); % Fecha a barra ao terminar
fprintf('\nConcluído!\n');

best_ax = ax_best / g;
best_ay = ay_best / g;

%% =========================================================
% RESULTS
% =========================================================
fprintf('MECHANICAL EVALUATION:\n');
fprintf('   1st Gear: %.2f%%\n', best_perc_1);
fprintf('   2nd Gear: %.2f%%\n', best_perc_2);
fprintf('-----------------------------------------------------------------------\n');
fprintf('Best Planetary Ratio (1st): %.4f\n', best_gear_planetary_first);
fprintf('Best Final Ratio     (2nd):        %.4f\n', best_gear_final);
fprintf('-----------------------------------------------------------------------\n');
fprintf('Minimum Lap-Time:            %.3f s\n', best_lap_time);
fprintf('Maximum Velocity:    %.2f km/h\n', max(v_best)*3.6);
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
fprintf('Battery Capacity (w/ %.0f%% margin): %.3f kWh\n', (safety_coeff/usable_fraction-1)*100, E_battery_kWh);
fprintf('-----------------------------------------------------------------------\n');
fprintf('Peak Power (wheel):          %.2f kW\n', P_peak_kW);
fprintf('Average Power:         %.2f kW\n', P_avg_kW);
fprintf('=======================================================================\n');

%% =========================================================
% PLOTS
% =========================================================

%% speed
figure('Name', 'Velocity and Forces Profile')

subplot(2,1,1)
plot(s, best_v_lim*3.6, '--', 'LineWidth', 1.2); hold on;
plot(s, v_best*3.6, 'LineWidth', 1.0);
grid on;
xlabel('s [m]');
ylabel('v [km/h]');
legend('v_{lim,curve}', 'v_{final}', 'Location', 'best');
title(sprintf('Speed profile') );

subplot(2,1,2)
plot(s, S_best, 'LineWidth', 1.0); hold on;
plot(s, F_drag_best,  'LineWidth', 1.0);
plot(s, F_rr_best,    'LineWidth', 1.0);
plot(s, F_slope_best,    'LineWidth', 1.0);
%plot(s, abs(F_lat_best), 'LineWidth', 1.0);
grid on;
xlabel('s [m]');
ylabel('Force [N]');
legend('F_{drive}', 'F_{drag}', 'F_{rr}', 'F_{slope}','Location', 'best');
title('Forces along track');

%% Power Consumption

t_acumulado = [0; cumsum(best_dt(1:end-1))];   % tempo acumulado [s]
P_seg = E_seg ./ best_dt;                      % potência média em cada segmento [W]
Energy_kWh = sum(E_seg) / (3600 * 1000);       % energia total por volta [kWh]

figure('Name', 'Análise de Potência e Energia');
area(t_acumulado, P_seg * 1e-3, 'FaceColor', [0.85 0.32 0.1], 'FaceAlpha', 0.3);
grid on;
ylabel('Power [kW]');
xlabel('Lap Time [s]');
title(sprintf('Power Profile [Time] (Total Energy: %.3f kWh)', Energy_kWh));
legend('Segment Power');

%% Long vs Lat acceleration

figure('Name', 'Lateral vs Longitudinal');
plot(s, best_ax, 'LineWidth', 1.0); hold on;
plot(s, best_ay, 'LineWidth', 1.0);
grid on;
xlabel('s [m]');
ylabel('a [g]');
legend('a_x', 'a_y', 'Location', 'best');
title('Lateral and Longitudinal Acceleration Distribution');

%% OPTIMAL COMBINATION SURFACE
[X, Y] = meshgrid(gear_ratio_final, gear_ratio_planetary_first);

figure('Name', 'Gear Ratio Optimization');
surf(X, Y, lap_time_vec); 
shading interp; colorbar; hold on;
grid on; view(45, 30);

% Labels and Titles in English
xlabel('Final Ratio');
ylabel('Planetary 1 (1st Gear)');
zlabel('Lap Time [s]');
title(['Performance Map (Laps: ', num2str(n_laps), ')']);

% Mark the optimal point in 3D space
plot3(best_gear_final, best_gear_planetary_first, best_lap_time, ...
    'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

%% diagrama de kamm

figure('Name', 'G-G Diagram');
plot(best_ay, best_ax, '.', 'MarkerSize', 2.5);
grid on;
xlabel('a_y [g]');
ylabel('a_x [g]');
title('G-G Diagram');
axis equal;

hold on;
ay_env = linspace(-mu, mu, 500);
ax_env_pos = sqrt(max(mu^2 - ay_env.^2, 0));
ax_env_neg = -ax_env_pos;

plot(ay_env, ax_env_pos, '--', 'LineWidth', 1.2);
plot(ay_env, ax_env_neg, '--', 'LineWidth', 1.2);

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