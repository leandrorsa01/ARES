
% --------------------------------------------------------------------- %
%                               Nível 3                                 %
%                        VOO INCLINADO NO VÁCUO                         %
%                             2 Estágios                                %
% --------------------------------------------------------------------- %

close all; clear; clc;

%% Parâmetros

% Conversões
deg2rad = pi/180;

% Constantes
Re = 6371008.8;                 % Raio médio da Terra
g0 = 9.80665;                   % Aceleração gravítica (nível no mar)
Isp1 = 302.3333;                % Impulso específico (motor principal)
Isp2 = 327.3333;                % Impulso específico (motor secundário)

% Condições iniciais
m1_0 = 41421.03468;             % Massa Húmida - 1º estágio
m2_0 = 3850.857442;             % Massa húmida - 2º estágio
gg0 = 89.5*deg2rad;             % Path Angle
h0 = 0;                         % Altitude relativa

% Estágio 1
T2W1 = 1.4;                     % Thrust to Weight ratio nominal
T1 = m1_0*g0*T2W1;              % Thrust (motor principal)
m_flux1 = T1 / (Isp1 * g0);     % Caudal mássico de prop
mf1 = 7119.462906;              % Massa final do primeiro estágio

% Estágio 2
T2W2 = 0.7;                     % Thrust to Weight ratio nominal
T2 = m2_0*g0*T2W2;              % Thrust (motor secundário)
m_flux2 = T2 / (Isp2 * g0);     % Caudal mássico de prop
mf2 = 757.1782087;              % Massa final do segundo estágio

% Eventos
h_kick = 200;                   % Altitude do Kick

%% Power / phase 0 - Vertical

x0_0 = [0; 0; 0.01; gg0; m1_0];
tspan0 = [0 100];
options0 = odeset('Events', @(t,x) kick(t, x, h_kick), ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t0, x0, t_kick, x_kick, i_kick] = ode45(@(t,x) ...
    movVerticalPower(t, x, g0, Re, Isp1, m_flux1), tspan0, x0_0, options0);

%% Power / phase 1 - Turn

tspan1 = [t_kick(end) t_kick(end)+200];
x1_0 = x0(end, :)';

options1 = odeset('Events', @(t, x) sensorCombustivel(t,x,mf1), ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t1, x1, t_meco, x_meco, i_meco] = ode45(@(t, x) ...
    movInclinadoPower(t, x, g0, Re, Isp1, m_flux1), tspan1, x1_0, options1);

%% Coast Until Separation / phase 2 - Turn

tspan2 = [t_meco(end) t_meco(end)+10];
x2_0 = x1(end,:)';

options2 = odeset('RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t2, x2] = ode45(@(t, x) ...
    movInclinadoCoast(t, x, g0, Re), tspan2, x2_0, options2);

%% Hipotetico

tspan_h = [t_meco(end) t_meco(end)+1000];
xh_0 = x2(end,:)';
xh_0(5) = m2_0;

optionsh = odeset('Events', @(t, x) coastEvents(t,x), ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[th, xh, t_coasth, x_coasth, i_coasth] = ode45(@(t, x) ...
    movInclinadoCoast(t, x, g0, Re), tspan_h, xh_0, optionsh);

%% Coast After Separation / phase 3 - Turn

tspan3 = [t2(end) t2(end)+100];
x3_0 = x2(end,:)';
x3_0(5) = m2_0;

options3 = options2;

[t3, x3] = ode45(@(t,x) ...
    movInclinadoCoast(t, x, g0, Re), tspan3, x3_0, options3);

%% Second Burn / phase 4 - Turn

tspan4 = [t3(end) t3(end)+1000];
x4_0 = x3(end,:)';

options4 = odeset('Events', @(t, x) sensorCombustivel(t,x,mf2), ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t4, x4, t_seco, x_seco, i_seco] = ode45(@(t,x) ...
    movInclinadoPower(t, x, g0, Re, Isp2, m_flux2), tspan4, x4_0, options4);

%% Coast / phase 5 - Turn

tspan5 = [t_seco(end) t_seco(end)+10000];
x5_0 = x4(end,:)';

options5 = odeset('Events', @(t, x) coastEvents(t,x), ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t5, x5, t_coast, x_coast, i_coast] = ode45(@(t, x) ...
    movInclinadoCoast(t, x, g0, Re), tspan5, x5_0, options5);

% ---------------------//-------------------- %

t = [t0; t1(2:end); t2(2:end); t3(2:end);
    t4(2:end); t5(2:end)];
x = [x0; x1(2:end,:); x2(2:end,:); x3(2:end,:);
    x4(2:end,:); x5(2:end,:)];

l = x(:,1);
h = x(:,2);
v = x(:,3);
gg = x(:,4);
m = x(:,5);

%% 4. RESULTADOS FINAIS 
fprintf('\n--- EVENTO KICK ---\n');
fprintf(['Tempo: %.4f s | Altitude: %.4f km ' ...
    '| Velocidade: %.4f m/s\n'], ...
    t0(end), x0(end,2)/1000, ...
    x0(end,3));

fprintf('\n--- EVENTO MECO ---\n');
fprintf(['Tempo: %.4f s | Longitude: %.4f m ' ...
    '| Altitude: %.4f km | Velocidade: %.4f m/s ' ...
    '| Path: %.4f deg\n'], ...
    t1(end), x1(end,1), ...
    x1(end,2)/1000, x1(end,3), ...
    x1(end,4)/deg2rad);

fprintf('\n--- EVENTO SEPARAÇÃO ---\n');
fprintf(['Tempo: %.4f s | Longitude: %.4f m ' ...
    '| Altitude: %.4f km | Velocidade: %.4f m/s ' ...
    '| Path: %.4f deg\n'], ...
    t2(end), x2(end,1), ...
    x2(end,2)/1000, x2(end,3), ...
    x2(end,4)/deg2rad);

fprintf('\n--- EVENTO SEI ---\n');
fprintf(['Tempo: %.4f s | Longitude: %.4f m ' ...
    '| Altitude: %.4f km | Velocidade: %.4f m/s ' ...
    '| Path: %.4f deg\n'], ...
    t3(end), x3(end,1), ...
    x3(end,2)/1000, x3(end,3), ...
    x3(end,4)/deg2rad);

fprintf('\n--- EVENTO SECO ---\n');
fprintf(['Tempo: %.4f s | Longitude: %.4f m ' ...
    '| Altitude: %.4f km | Velocidade: %.4f m/s ' ...
    '| Path: %.4f deg\n'], ...
    t4(end), x4(end,1), ...
    x4(end,2)/1000, x4(end,3), ...
    x4(end,4)/deg2rad);

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
plot(xh(:,1)/1000, xh(:,2)/1000, 'g--', 'LineWidth', 1);
plot(l(end)/1000,h(end)/1000, 'rx', 'MarkerSize', 8, 'LineWidth', 2);
plot(x3(end,1)/1000, x3(end,2)/1000, 'go', 'MarkerSize', 10, 'LineWidth', 1);
xlabel('Downrange - Distância Horizontal (km)');
ylabel('Altitude (km)');
title('Trajetória 2D: O Gravity Turn');
grid on; axis equal;

figure('Name', 'Perfil de Voo: Nível 2', 'Position', [100, 100, 800, 600]);

subplot(4,1,1);
plot(t, h/1000, 'b', 'LineWidth', 1.5);
ylabel('Altitude (km)'); title('Trajetória Vertical (Vácuo)'); grid on;

subplot(4,1,2);
plot(t, v, 'm', 'LineWidth', 1.5); % 'm' de magenta para contrastar
ylabel('Vel. Absoluta (m/s)'); grid on;

subplot(4,1,3);
plot(t, v.*sin(gg), 'r', 'LineWidth', 1.5);
ylabel('Velocidade Vertical (m/s)'); grid on;

subplot(4,1,4);
plot(t, m, 'k', 'LineWidth', 1.5);
ylabel('Massa (kg)'); xlabel('Tempo (s)'); grid on;



