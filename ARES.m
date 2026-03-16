
% --------------------------------------------------------------------- %
%                           Nível 2                                     %
%                    VOO INCLINADO NO VÁCUO                              %
% --------------------------------------------------------------------- %

close all; clear; clc;

%% Parâmetros

deg2rad = pi/180;

Re = 6371008.8;
g0 = 9.80665;
m0 = 41421.03468;
mf1 = 7119.462906;
gg0 = 89.5*deg2rad;
h0 = 0;
Isp1 = 302.3333;
T2Wr0 = 1.4;
T0 = m0*g0*T2Wr0;
m_flux = T0 / (Isp1 * g0);
h_kick = 200;

%% Power / phase 0 - Vertical

x0_0 = [0; 0; 0.01; gg0; m0];
tspan0 = [0 100];
options0 = odeset('Events', @(t,x) kick(t, x, h_kick), ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t0, x0, t_kick, x_kick, i_kick] = ode45(@(t,x) ...
    movVerticalPower(t, x, g0, Re, Isp1, m_flux), tspan0, x0_0, options0);

%% Power / phase 1 - Turn

tspan1 = [t_kick(end) t_kick(end)+200];
x1_0 = x0(end, :)';

options1 = odeset('Events', @(t, x) sensorCombustivel(t,x,mf1), ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t1, x1, t_meco, x_meco, i_meco] = ode45(@(t, x) ...
    movInclinadoPower(t, x, g0, Re, Isp1, m_flux), tspan1, x1_0, options1);

%% Coast / phase 2 - Turn

tspan2 = [t_meco(end) t_meco(end)+2000];
x2_0 = x1(end,:)';

options2 = odeset('Events', @(t, x) coastEvents(t,x), ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t2, x2, t_coast, x_coast, i_coast] = ode45(@(t, x) ...
    movInclinadoCoast(t, x, g0, Re), tspan2, x2_0, options2);

% ---------------------//-------------------- %

t = [t0; t1(2:end); t2(2:end)];
x = [x0; x1(2:end,:); x2(2:end,:)];

l = x(:,1);
h = x(:,2);
v = x(:,3);
gg = x(:,4);
m = x(:,5);

%% 4. RESULTADOS FINAIS 
fprintf('\n--- EVENTO KICK ---\n');
fprintf('Tempo (tk): %.4f s | Altitude (h_k): %.4f km | Velocidade (vk): %.4f m/s\n', t0(end), x0(end,2)/1000, x0(end,3));

fprintf('\n--- EVENTO MECO ---\n');
fprintf(['Tempo (tb): %.4f s | Longitude (xb) = %.4f m ' ...
    '| Altitude (hb): %.4f km | Velocidade (vb): %.4f m/s ' ...
    '| Path (ggb) = %.4f deg\n'], ...
    t1(end), x1(end,1), ...
    x1(end,2)/1000, x1(end,3), ...
    x1(end,4)/deg2rad);

i_apogeu = find(i_coast == 1);
if ~isempty(i_apogeu)
    fprintf('\n--- EVENTO APOGEU REGISTADO ---\n');
    fprintf(['Tempo (t_apo): %.4f s | Longitude (x_apo): %.4f m ' ...
        '| Altitude (h_apo) = %.4f km | Velocidade (v_apo): %.4f m/s ' ...
        '| Path (gg_apo) = %.4f deg\n'], ...
        t_coast(i_apogeu), x_coast(i_apogeu, 1), ...
        x_coast(i_apogeu,2)/1000, x_coast(i_apogeu,3), ...
        x_coast(i_apogeu,4)/deg2rad);
end

i_ground = find(i_coast == 2);
if ~isempty(i_ground)
    fprintf('\n--- EVENTO GROUND REGISTADO ---\n');
    fprintf('Tempo: %.4f s | Velocidade: %.4f m/s\n', ...
        t_coast(i_ground), x_coast(i_ground,3));
end

% Gráficos
figure('Name','Gravity Turn', 'Position', [100, 100, 900, 500]);
plot(l/1000, h/1000, 'b', 'LineWidth', 1.3);
hold on;
plot(l(end)/1000,h(end)/1000, 'rx', 'MarkerSize', 10, 'LineWidth', 1);
xlabel('Downrange - Distância Horizontal (km)');
ylabel('Altitude (km)');
title('Trajetória 2D: O Gravity Turn');
grid on; axis equal;

figure('Name', 'Perfil de Voo: Nível 2', 'Position', [100, 100, 800, 600]);

subplot(3,1,1);
plot(t, h/1000, 'b', 'LineWidth', 1.5);
ylabel('Altitude (km)'); title('Trajetória Vertical (Vácuo)'); grid on;

subplot(3,1,2);
plot(t, v.*sin(gg), 'r', 'LineWidth', 1.5);
ylabel('Velocidade (m/s)'); grid on;

subplot(3,1,3);
plot(t, m, 'k', 'LineWidth', 1.5);
ylabel('Massa (kg)'); xlabel('Tempo (s)'); grid on;



