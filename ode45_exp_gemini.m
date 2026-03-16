% =========================================================================
% SIMULADOR DE TRAJETÓRIA - NÍVEL 1: Voo Vertical 1D no Vácuo
% Projeto: Micro-lançador (Iteração 0)
% =========================================================================
clear; clc; close all;

%% 1. DEFINIÇÃO DOS PARÂMETROS FÍSICOS E DO VEÍCULO
g0 = 9.80665;               % Aceleração da gravidade à superfície (m/s^2)

% Os teus dados da Iteração 0:
m0 = 41421.04;              % Massa Inicial / GLOW (kg)
mf_1 = 7119.46;             % Massa Final do 1º Andar no momento da separação (kg)
Isp_1 = 302.3;              % Impulso Específico do 1º Andar (s)
TWR = 1.3;                  % Thrust-to-Weight Ratio escolhido para arrancar

% Cálculos derivados do motor:
T0 = m0 * g0 * TWR;         % Força de Empuxo (Newtons) -> aprox. 528 kN
m_dot = T0 / (Isp_1 * g0);  % Caudal Mássico (kg/s) -> aprox. 178 kg/s

fprintf('--- INÍCIO DA SIMULAÇÃO (NÍVEL 1) ---\n');
fprintf('Empuxo do Motor: %.1f kN\n', T0/1000);
fprintf('Caudal Mássico: %.2f kg/s\n', m_dot);

%% 2. CONDIÇÕES INICIAIS E CONFIGURAÇÃO DO ODE45
% O nosso Vetor de Estado inicial: [Altitude(m); Velocidade(m/s); Massa(kg)]
Y0 = [0; 0; m0];

% Tempo máximo que damos ao simulador (ele vai parar antes por causa do evento)
tspan = [0 250]; 

% Dizemos ao MATLAB: "Fica atento à função EventoCorteMotor. Se ela apitar, para!"
opcoes = odeset('Events', @(t,Y) EventoCorteMotor(t, Y, mf_1));

%% 3. A INTEGRAÇÃO (O Motor do Simulador)
% Chamamos o ode45 para resolver a nossa função da Física
[tempo, Y, t_evento, Y_evento, i_evento] = ode45(@(t,Y) FisicaVooVertical(t, Y, T0, m_dot, g0), tspan, Y0, opcoes);

% Extrair os resultados para variáveis mais fáceis de ler
h = Y(:, 1);  % Coluna 1: Altitude
v = Y(:, 2);  % Coluna 2: Velocidade
m = Y(:, 3);  % Coluna 3: Massa

%% 4. RESULTADOS FINAIS E PLOTS
fprintf('\n--- RESULTADOS DO MECO (Corte do Motor 1) ---\n');
fprintf('Tempo de Queima (Burn Time): %.2f segundos\n', tempo(end));
fprintf('Altitude Atingida: %.2f km\n', h(end)/1000);
fprintf('Velocidade Atingida: %.2f m/s\n', v(end));

% Criar os gráficos para analisares a física
figure('Name', 'Perfil de Voo: Nível 1', 'Position', [100, 100, 800, 600]);

subplot(3,1,1);
plot(tempo, h/1000, 'b', 'LineWidth', 2);
ylabel('Altitude (km)'); title('Trajetória Vertical (Vácuo)'); grid on;

subplot(3,1,2);
plot(tempo, v, 'r', 'LineWidth', 2);
ylabel('Velocidade (m/s)'); grid on;

subplot(3,1,3);
plot(tempo, m, 'k', 'LineWidth', 2);
ylabel('Massa (kg)'); xlabel('Tempo (s)'); grid on;


%% =========================================================================
% AS FUNÇÕES DO ODE45 (Devem ficar sempre no fim do script)
% =========================================================================

function dYdt = FisicaVooVertical(t, Y, T, m_dot, g)
    % Esta função diz ao ode45 como as coisas mudam num instante 't'
    
    % Ler as variáveis atuais da "gaveta" Y
    h_atual = Y(1); 
    v_atual = Y(2);
    m_atual = Y(3);
    
    % Preparar a resposta (3 derivadas)
    dYdt = zeros(3,1);
    
    % 1. A derivada da altitude é a velocidade (dh/dt = v)
    dYdt(1) = v_atual;
    
    % 2. A derivada da velocidade é a aceleração (2ª Lei de Newton: a = F/m - g)
    dYdt(2) = (T / m_atual) - g;
    
    % 3. A derivada da massa é o caudal negativo (dm/dt = -mdot)
    dYdt(3) = -m_dot;
end

function [value, isterminal, direction] = EventoCorteMotor(t, Y, mf_alvo)
    % O ode45 avalia isto a cada passo. Quando 'value' for zero, o evento dispara.
    m_atual = Y(3);
    
    value = m_atual - mf_alvo; % Subtrai a massa atual da massa final alvo
    isterminal = 1;            % 1 = Sim, pára a simulação completamente!
    direction = 0;             % 0 = Aproximação de qualquer direção
end