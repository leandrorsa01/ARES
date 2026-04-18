
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
                                   
Veiculo = loadVeiculo(0);
% Variáveis dinâmicas ótimas do Veículo
Veiculo.h_pitchOver = 199;                                      % Altitude do Kick
Veiculo.gg0 = 89.5*deg2rad;                                     % Path Angle

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
    t2(end), 100, x3_0, []);

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

%% RESULTADOS FINAIS 
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



