% --------------------------------------------------------------------- %
%                                  V2                                   %
%                                 6DoF                                  %
%                             2 Estágios                                %
% --------------------------------------------------------------------- %

clear; close all; clc;

%% Parâmetros

% Constantes Planeta
Planeta.Re = 6371008.8;                          % Raio médio da Terra
Planeta.g0 = 9.80665;                            % Aceleração gravítica (nível no mar)
Planeta.h0 = 0;                                  % Altitude relativa

% Constantes Missão
Missao.h_max = 800000;
Missao.t_sep = 10;

% Veiculo
Veiculo = loadVeiculo_V2(3);

%% Execução das Fases de Voo
% Phase 0 - Power Vertical
Pos0   = [0; 0; Planeta.h0];
Quat0  = [0.5; 0.5; -0.5; 0.5];
Vel0   = [0; 0; 0];
Rot0   = [0; 0; 0];
Mflux0 = [Veiculo.m_RP1_S1 + Veiculo.m_RP1_S2;
          Veiculo.m_LOX_S1 + Veiculo.m_LOX_S2];

x0_0 = [Pos0; Quat0; Vel0; Rot0; Mflux0];
ev0 = @(t,x) gestorEventos(t,x,Planeta,'pitchOver',Veiculo.h_pitchOver);
options = odeset('Events', ev0, 'RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', 0.1);
tspan0 = [0 50];
[t0, x0, te0, xe0, ie0] = ode45(@(t,x) EoM(t, x, Planeta, Veiculo, 0, 0, 1,1), ...
    tspan0, x0_0, options);
verificaRestricoes(ie0);

% Phase 1 - Pitch Over
ev1 = @(t,x) gestorEventos(t,x,Planeta);
options = odeset('Events', ev1, 'RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', 0.1);
tspan1 = [t0(end), t0(end) + 1];
[t1, x1, te1, xe1, ie1] = ode45(@(t,x) EoM(t,x,Planeta,Veiculo,0,2,1,1), ...
    tspan1,x0(end,:)',options);
verificaRestricoes(ie1);

% Phase 2 - Power Inclined
ev2 = @(t,x) gestorEventos(t,x,Planeta,'combustivel',Veiculo.m_RP1_S2+Veiculo.m_LOX_S2);
options = odeset('Events', ev2, 'RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', 0.1);
tspan2 = [t1(end), t1(end) + 300];
[t2, x2, te2, xe2, ie2] = ode45(@(t,x) EoM(t, x, Planeta, Veiculo, 0, 0, 1,1), ...
    tspan2, x1(end,:)', options);
verificaRestricoes(ie2);

% Phase 3 - Coast til separation
ev3 = @(t,x) gestorEventos(t,x,Planeta);
options = odeset('Events', ev3, 'RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', 0.1);
tspan3 = [t2(end), t2(end) + Veiculo.t_sei];
[t3, x3, te3, xe3, ie3] = ode45(@(t,x) EoM(t,x,Planeta,Veiculo,0,0,1,0), ...
    tspan3,x2(end,:)',options);
verificaRestricoes(ie3);

% Phase 4 - Burn til apogeu=800km
ev4 = @(t,x) gestorEventos(t,x,Planeta,'apogeuProjetado',800);
options = odeset('Events', ev4, 'RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', 0.1);
tspan4 = [t3(end), t3(end) + 1000];
[t4, x4, te4, xe4, ie4] = ode45(@(t,x) EoM(t, x, Planeta, Veiculo, 0, 0, 2,1), ...
    tspan4, x3(end,:)', options);
verificaRestricoes(ie4);

% Phase 5 - Coast til 800km
ev5 = @(t,x) gestorEventos(t,x,Planeta,'apogeuSensor',800);
options = odeset('Events', ev5, 'RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', 0.1);
tspan5 = [t4(end), t4(end) + 1000];
[t5, x5, te5, xe5, ie5] = ode45(@(t,x) EoM(t, x, Planeta, Veiculo, 0, 0, 2,0), ...
    tspan5, x4(end,:)', options);
verificaRestricoes(ie5);

% Phase 6 - Circularization
ev6 = @(t,x) gestorEventos(t,x,Planeta,'circularizacao',800);
options = odeset('Events', ev6, 'RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', 0.1);
tspan6 = [t5(end), t5(end) + 100];
[t6, x6, te6, xe6, ie6] = ode45(@(t,x) EoM(t, x, Planeta, Veiculo, 0, 0, 2,1), ...
    tspan6, x5(end,:)', options);
verificaRestricoes(ie6);

% Phase 7 - Coast for confirmation
ev7 = @(t,x) gestorEventos(t,x,Planeta);
options = odeset('Events', ev7, 'RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', 0.1);
tspan7 = [t6(end), t6(end) + 300];
[t7, x7, te7, xe7, ie7] = ode45(@(t,x) EoM(t,x,Planeta,Veiculo,0,0,2,0), ...
    tspan7,x6(end,:)',options);
verificaRestricoes(ie7);

%% Processamento e Concatenação de Dados

t = [t0; t1(2:end); t2(2:end); t3(2:end); t4(2:end); 
     t5(2:end); t6(2:end); t7(2:end)];

x = [x0; x1(2:end, :); x2(2:end, :); x3(2:end, :); x4(2:end, :); 
     x5(2:end, :); x6(2:end, :); x7(2:end, :)];

X_pos = x(:,1);
Y_pos = x(:,2);
Z_pos = x(:,3);

q0 = x(:,4);
q1 = x(:,5);
q2 = x(:,6);
q3 = x(:,7);

Roll_deg  = atan2(2.*(q0.*q1 + q2.*q3), 1 - 2.*(q1.^2 + q2.^2)) * (180/pi);
Pitch_deg = asin(2.*(q0.*q2 - q3.*q1)) * (180/pi);
Yaw_deg   = atan2(2.*(q0.*q3 + q1.*q2), 1 - 2.*(q2.^2 + q3.^2)) * (180/pi);

% 1. Gráfico da Trajetória 3D
figure('Name', 'Trajetória 3D', 'Color', 'w');
plot3(X_pos/1000, Y_pos/1000, Z_pos/1000, 'b', 'LineWidth', 2);
grid on;
xlabel('Eixo X - East (km)');
ylabel('Eixo Y - North (km)');
zlabel('Eixo Z - Altitude (km)');
title('Trajetória Orbital 3D - ESPERANÇA-I');
view(45, 30); % Ângulo de visualização isométrico

% 2. Gráficos de Atitude (Cinemática de Rotação)
figure('Name', 'Atitude do Veículo', 'Color', 'w');

subplot(3,1,1);
plot(t, Pitch_deg, 'r', 'LineWidth', 1.5);
grid on;
ylabel('Arfagem (Pitch) [deg]');
title('Evolução da Atitude (Ângulos de Euler)');

subplot(3,1,2);
plot(t, Yaw_deg, 'g', 'LineWidth', 1.5);
grid on;
ylabel('Guinada (Yaw) [deg]');

subplot(3,1,3);
plot(t, Roll_deg, 'b', 'LineWidth', 1.5);
grid on;
ylabel('Rolamento (Roll) [deg]');
xlabel('Tempo de Missão (s)');

%% ========================================================================
%  Animação 3D (Chase Cam) - O Momento da Verdade
% =========================================================================
figure('Name', 'Simulação Visual 6-DOF', 'Color', 'w', 'Position', [100, 100, 800, 600]);

% 1. Desenhar a Trajetória completa no fundo
plot3(X_pos, Y_pos, Z_pos, 'k--', 'LineWidth', 0.5); 
hold on; grid on; axis equal;
xlabel('East - X (m)'); ylabel('North - Y (m)'); zlabel('Altitude - Z (m)');
view(45, 20); % Ângulo da câmara inicial

% 2. Criar o objeto de Transformação Dinâmica (Nome alterado para evitar conflitos)
modelo_3D = hgtransform('Parent', gca);

% 3. Modelar o Foguetão (Orientado no Eixo X local)
Raio = Veiculo.d / 2;
Comp = Veiculo.h_S1 + Veiculo.h_S2; % Comprimento total aproximado

% Fuselagem (Cilindro branco)
[C_X, C_Y, C_Z] = cylinder(Raio, 30);
surf(C_Z * Comp, C_Y, C_X, 'Parent', modelo_3D, 'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none');

% Nariz (Cone vermelho)
[N_X, N_Y, N_Z] = cylinder([Raio, 0], 30);
surf(N_Z * 3 + Comp, N_Y, N_X, 'Parent', modelo_3D, 'FaceColor', 'r', 'EdgeColor', 'none');

% Iluminação para parecer 3D
camlight right; lighting gouraud;

% 4. Motor de Animação (Com Controlo de Tempo e Slow-Motion)
FramesTotais = 3500; % Resolução visual
passo = max(1, floor(length(t) / FramesTotais)); 
zoom_box = 50; % A câmara mostra uma caixa de 50 metros à volta do foguetão

% --- CONTROLOS DO TEMPO ---
FPS = 60; % Velocidade base de reprodução (frames por segundo)
SlowMo = 4; % 1 = Normal, 2 = Metade da velocidade, 4 = Câmara super lenta
tempo_espera = (1 / FPS) * SlowMo;

disp('A preparar animação 3D... A simulação começa em 3 segundos. MAXIMIZA A JANELA!');
pause(3); 

for k = 1:passo:length(t)
    % Extrair Posição
    px = X_pos(k); py = Y_pos(k); pz = Z_pos(k);
    
    % Extrair Quaterniões
    q_0 = q0(k); q_1 = q1(k); q_2 = q2(k); q_3 = q3(k);
    
    % Converter Quaterniões DIRETAMENTE para Matriz de Rotação
    R = [1 - 2*(q_2^2 + q_3^2),  2*(q_1*q_2 - q_0*q_3),  2*(q_1*q_3 + q_0*q_2);
         2*(q_1*q_2 + q_0*q_3),  1 - 2*(q_1^2 + q_3^2),  2*(q_2*q_3 - q_0*q_1);
         2*(q_1*q_3 - q_0*q_2),  2*(q_2*q_3 + q_0*q_1),  1 - 2*(q_1^2 + q_2^2)];
         
    % Construir a Matriz de Transformação Homogénea (4x4)
    M = eye(4);
    M(1:3, 1:3) = R;               % Rotação
    M(1:3, 4)   = [px; py; pz];    % Translação
    
    % Aplicar ao modelo 3D (usando a nossa variável correta)
    set(modelo_3D, 'Matrix', M);
    
    % Atualizar os limites do gráfico (A câmara segue o foguetão)
    axis([px-zoom_box, px+zoom_box, py-zoom_box, py+zoom_box, pz-zoom_box, pz+zoom_box]);
    
    % Titulo dinâmico
    title(sprintf('Tempo: %.1f s | Altitude: %.1f km | Velocidade Visual: %dx', t(k), pz/1000, 1/SlowMo));
    
    % Forçar o MATLAB a renderizar a frame
    drawnow;
    
    % Travão de tempo
    pause(tempo_espera);
end

disp('Animação concluída!');




































