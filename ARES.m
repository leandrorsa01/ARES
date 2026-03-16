
% --------------------------------------------------------------------- %
%                           Nível 2                                     %
%                    VOO VERTICAL NO VÁCUO                              %
% --------------------------------------------------------------------- %

close all; clear; clc;

%% Parâmetros

Re = 6371008.8;
g0 = 9.80665;
m0 = 41421.03468;
mf1 = 7119.462906;
h0 = 0;
Isp1 = 302.3333;
T2Wr0 = 1.4;
T0 = m0*g0*T2Wr0;
m_flux = T0 / (Isp1 * g0);

%% Power phase 0

x0_0 = [0; 0; m0];
tspan0 = [0 250];
options0 = odeset('Events', @(t,x) sensorCombustivel(t, x, mf1));

[t0, x0, t_meco, x_meco, i_meco] = ode45(@(t,x) ...
    movVerticalPower(t, x, g0, Re, Isp1, m_flux), tspan0, x0_0, options0);

%% Coast phase 1

tspan1 = [t_meco(end) t_meco(end)+1000];
x1_0 = x0(end, :)';

options1 = odeset('Events', @(t, x) coastEvents(t,x));

[t1, x1, t_coast, x_coast, i_coast] = ode45(@(t, x) ...
    movVerticalCoast(t, x, g0, Re), tspan1, x1_0, options1);

t = [t0; t1(2:end)];
x = [x0; x1(2:end,:)];

v = x(:,1);
h = x(:,2);
m = x(:,3);

%% 4. RESULTADOS FINAIS 
fprintf('\n--- EVENTO MECO ---\n');
fprintf('Tempo (tb): %.4f s | Altitude (h_meco): %.4f km | Velocidade (vb): %.4f m/s\n', t0(end), x0(end,2)/1000, x0(end,1));

i_apogeu = find(i_coast == 1);
if ~isempty(i_apogeu)
    fprintf('\n--- EVENTO APOGEU REGISTADO ---\n');
    fprintf('Tempo (t_apo): %.4f s | Altitude (h_apo): %.4f km | Velocidade (v_apo): %.4f m/s\n', t_coast(i_apogeu), x_coast(i_apogeu, 2)/1000, x_coast(i_apogeu,1));
end

i_ground = find(i_coast == 2);
if ~isempty(i_ground)
    fprintf('\n--- EVENTO GROUND REGISTADO ---\n');
    fprintf('Tempo: %.4f s | Altitude: %.4f km | Velocidade: %.4f m/s\n', t_coast(i_ground), x_coast(i_ground, 2)/1000, x_coast(i_ground,1));
end

% Gráficos
figure('Name', 'Perfil de Voo: Nível 1', 'Position', [100, 100, 800, 600]);

subplot(3,1,1);
plot(t, h/1000, 'b', 'LineWidth', 1.5);
ylabel('Altitude (km)'); title('Trajetória Vertical (Vácuo)'); grid on;

subplot(3,1,2);
plot(t, v, 'r', 'LineWidth', 1.5);
ylabel('Velocidade (m/s)'); grid on;

subplot(3,1,3);
plot(t, m, 'k', 'LineWidth', 1.5);
ylabel('Massa (kg)'); xlabel('Tempo (s)'); grid on;


