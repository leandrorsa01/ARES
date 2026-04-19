
% --------------------------------------------------------------------- %
%                               Nível 5                                 %
%               VOO INCLINADO NA ATMOSFÉRA - Circulização               %
%                             2 Estágios                                %
% --------------------------------------------------------------------- %

close all; clear; clc;

%% Parâmetros

% Conversões
deg2rad = pi/180;

% Constantes Planeta
Planeta.Re = 6371008.8;                                         % Raio médio da Terra
Planeta.g0 = 9.80665;                                           % Aceleração gravítica (nível no mar)
Planeta.h0 = 0;                                                 % Altitude relativa
                                   
Veiculo = loadVeiculo(1);

%% Execução das Fases de Voo
% Phase 0 - Power Vertical

x0_0 = [0; 0; 0.01; Veiculo.gg0; Veiculo.m1_0];
ev0 = @(t,x) superEvento(t, x, Planeta, 'pitchOver', Veiculo.h_pitchOver);
[t0, x0, t_kick, x_kick] = executarFase(@(t,x) movVerticalPower(t, x, Planeta, Veiculo), ...
    0, 100, x0_0, ev0);

% Phase 1 - Power Inclined

ev1 = @(t, x) superEvento(t,x,Planeta, 'sensorCombustivel', Veiculo.mf1);
[t1, x1, t_meco, x_meco] = executarFase(@(t, x) movInclinadoPower(t, x, Planeta, Veiculo, 1), ...
    t0(end), 200, x0(end,:)', ev1);

% Phase 2 - Coast Until Separation Inclined

[t2, x2] = executarFase(@(t, x) movInclinadoCoast(t, x, Planeta, Veiculo), ...
    t1(end), 10, x1(end,:)', []);

% Hipotetico ---------------------------------------------------------- %
                                                                         
xh_0 = x2(end,:)'; xh_0(5) = Veiculo.m2_0;
[th, xh] = executarFase(@(t, x) movInclinadoCoast(t, x, Planeta, Veiculo), ...
    t2(end), 1000, xh_0, @(t, x) coastEvents(t,x));
% ---------------------------------------------------------------------- %

% Phase 3 - Coast After Separation Inclined

x3_0 = x2(end,:)'; x3_0(5) = Veiculo.m2_0;
[t3, x3] = executarFase(@(t,x) movInclinadoCoast(t, x, Planeta, Veiculo), ...
    t2(end), Veiculo.t_sei, x3_0, []);

% Phase 4 - Second Burn Inclined

ev4 = @(t, x) superEvento(t,x,Planeta,'apogeuProjetado',800);
[t4, x4, t_seco, x_seco] = executarFase(@(t,x) movInclinadoPower(t, x, Planeta, Veiculo, 2), ...
    t3(end), 1000, x3(end,:)', ev4);

% Phase 5 - Coast Inclined

ev5 = @(t, x) superEvento(t,x,Planeta,'sensorApogeu',0);
[t5, x5, t_apogeu, x_apogeu] = executarFase(@(t, x) movInclinadoCoast(t, x, Planeta, Veiculo), ...
    t4(end), 10000, x4(end,:)', ev5);

% Phase 6 - Circularization Inclined

ev6 = @(t, x) superEvento(t, x, Planeta, 'perigeuProjetado', 800);
[t6, x6, t_circ, x_circ] = executarFase(@(t,x) movInclinadoPower(t, x, Planeta, Veiculo, 2), t5(end), 500, x5(end,:)', ev6);

%% Concatenação de Dados

t = [t0; t1(2:end); t2(2:end); t3(2:end);
    t4(2:end); t5(2:end); t6(2:end,:)];
x = [x0; x1(2:end,:); x2(2:end,:); x3(2:end,:);
    x4(2:end,:); x5(2:end,:); x6(2:end,:)];
l = x(:,1); h = x(:,2); v = x(:,3); gg = x(:,4); m = x(:,5);

%% RESULTADOS DINÂMICA DE VOO 
imprimeEvento('Pitch-Over', t0(end), x0(end,:));
imprimeEvento('MECO', t1(end), x1(end,:));
imprimeEvento('Separação', t2(end), x2(end,:));
imprimeEvento('SEI', t3(end), x3(end,:));
imprimeEvento('SECO', t4(end), x4(end,:));
imprimeEvento('Apogeu', t5(end), x5(end,:));
imprimeEvento('Fim da Circularização', t6(end), x6(end,:));

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
plot(t, v, 'm', 'LineWidth', 1.5); 
ylabel('Vel. Absoluta (m/s)'); grid on;
subplot(4,1,3);
plot(t, v.*sin(gg), 'r', 'LineWidth', 1.5);
ylabel('Velocidade Vertical (m/s)'); grid on;
subplot(4,1,4);
plot(t, m, 'k', 'LineWidth', 1.5);
ylabel('Massa (kg)'); xlabel('Tempo (s)'); grid on;

%% 5. Análise Aerodinâmica: O Max Q

rho = zeros(length(h), 1);
q = zeros(length(h), 1);

for i = 1:length(h) 
    [rho(i), ~, ~, ~] = atmosfera_100km(h(i)); 
    
    % Pressão Dinâmica (q = 1/2 * rho * v^2) em Pascals
    q(i) = 0.5 * rho(i) * (v(i)^2);
end

[max_q_val, idx_max_q] = max(q);
t_max_q = t(idx_max_q);
h_max_q = h(idx_max_q);
v_max_q = v(idx_max_q);

fprintf('\n--- ANÁLISE DE MAX Q (PRESSÃO DINÂMICA) ---\n');
fprintf('Max Q: %.2f Pa (%.3f bar)\n', max_q_val, max_q_val/1e5);
fprintf('Ocorre aos: %.1f s\n', t_max_q);
fprintf('Altitude do Max Q: %.1f km\n', h_max_q/1000);
fprintf('Velocidade no Max Q: Mach %.2f\n', v_max_q/340); % Assumindo vel. som ~340 m/s

figure('Name', 'Perfil de Pressão Dinâmica', 'Color', 'w');
plot(t, q / 1000, 'r', 'LineWidth', 1.5); hold on;
plot(t_max_q, max_q_val / 1000, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
text(t_max_q + 5, max_q_val / 1000, sprintf('Max Q: %.1f kPa\n(%.1f km)', max_q_val/1000, h_max_q/1000), 'FontSize', 11, 'FontWeight', 'bold');
% Encontrar o tempo em que o foguetão cruza a linha de Karman
idx_karman = find(h >= 100000, 1);
if ~isempty(idx_karman)
    xlim([0, t(idx_karman)]); 
else
    xlim([0, t_max_q * 3]);
end
xlabel('Tempo de Voo (s)');
ylabel('Pressão Dinâmica - q (kPa)');
title('Perfil de Pressão Dinâmica (Fase Atmosférica)');
grid on;

%% 6. Análise de Perdas de Velocidade

% 1. PERDAS POR ARRASTO (Drag Losses)
Drag = q * Veiculo.Aref * Veiculo.CD;
a_drag = Drag ./ m;

dV_drag = trapz(t, a_drag);

% 2. PERDAS POR GRAVIDADE (Gravity Losses)
g_local = Planeta.g0 * (Planeta.Re ./ (Planeta.Re + h)).^2;
a_grav = g_local .* sin(gg);

dV_grav = trapz(t, a_grav);

fprintf('\n--- ANÁLISE DE PERDAS DE DELTA-V ---\n');
fprintf('Perdas por Arrasto:    %4.1f m/s\n', dV_drag);
fprintf('Perdas por Gravidade:  %4.1f m/s\n', dV_grav);
fprintf('------------------------------------\n');
fprintf('PERDAS TOTAIS: %4.1f m/s\n', dV_drag + dV_grav);




























