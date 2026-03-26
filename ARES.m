
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
Planeta.rho0 = 1.225;                                           % Massa volúmica do ar padrão (nível do mar)
Planeta.h0 = 0;                                                 % Altitude relativa
Planeta.H_0 = 7640;                                             % Altura de escala

% Estados Iniciais Veiculo
Veiculo.d = 1.5;                                                % Diâmetro do Veiculo
Veiculo.Aref = pi * (Veiculo.d/2)^2;                            % Área de Referência
Veiculo.CD = 0.2;                                               % Coeficiente de arrasto utopico
Veiculo.gg0 = 89.5*deg2rad;                                     % Path Angle

% Estágio 1
Veiculo.m1_0 = 41421.03468;                                     % Massa Húmida - 1º estágio
Veiculo.mf1 = 7119.462906;                                      % Massa final do primeiro estágio
Veiculo.Isp1 = 302.3333;                                        % Impulso específico (motor principal)
Veiculo.T2W1 = 1.4;                                             % Thrust to Weight ratio nominal
Veiculo.T1 = Veiculo.m1_0*Planeta.g0*Veiculo.T2W1;              % Thrust (motor principal)
Veiculo.m_flux1 = Veiculo.T1 / (Veiculo.Isp1 * Planeta.g0);     % Caudal mássico de prop

% Estágio 2
Veiculo.m2_0 = 3850.857442;                                     % Massa húmida - 2º estágio
Veiculo.mf2 = 757.1782087;                                      % Massa final do segundo estágio
Veiculo.Isp2 = 327.3333;                                        % Impulos específico (motor secundário)
Veiculo.T2W2 = 0.7;                                             % Thrust to Weight ratio nominal
Veiculo.T2 = Veiculo.m2_0*Planeta.g0*Veiculo.T2W2;              % Thrust (motor secundário)
Veiculo.m_flux2 = Veiculo.T2 / (Veiculo.Isp2 * Planeta.g0);     % Caudal mássico de prop

% Eventos Veiculo
Veiculo.h_kick = 200;                                           % Altitude do Kick

%% Power / phase 0 - Vertical

x0_0 = [0; 0; 0.01; Veiculo.gg0; Veiculo.m1_0];
tspan0 = [0 100];
options0 = odeset('Events', @(t,x) kick(t, x, Veiculo.h_kick), ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t0, x0, t_kick, x_kick, i_kick] = ode45(@(t,x) ...
    movVerticalPower(t, x, Planeta, Veiculo), tspan0, x0_0, options0);

%% Power / phase 1 - Turn

tspan1 = [t_kick(end) t_kick(end)+200];
x1_0 = x0(end, :)';

options1 = odeset('Events', @(t, x) sensorCombustivel(t,x,Veiculo.mf1), ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t1, x1, t_meco, x_meco, i_meco] = ode45(@(t, x) ...
    movInclinadoPower(t, x, Planeta, Veiculo, 1), tspan1, x1_0, options1);

%% Coast Until Separation / phase 2 - Turn

tspan2 = [t_meco(end) t_meco(end)+10];
x2_0 = x1(end,:)';

options2 = odeset('RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t2, x2] = ode45(@(t, x) ...
    movInclinadoCoast(t, x, Planeta, Veiculo), tspan2, x2_0, options2);

%% Hipotetico ---------------------------------------------------------- %
                                                                         %
tspan_h = [t_meco(end) t_meco(end)+1000];                                %
xh_0 = x2(end,:)';                                                       %
xh_0(5) = Veiculo.m2_0;                                                  %
                                                                         %
optionsh = odeset('Events', @(t, x) coastEvents(t,x), ...                %
    'RelTol', 1e-6, ...                                                  %
    'AbsTol', 1e-9, ...                                                  %
    'MaxStep', 0.1);                                                     %
                                                                         %
[th, xh, t_coasth, x_coasth, i_coasth] = ode45(@(t, x) ...               %
    movInclinadoCoast(t, x, Planeta, Veiculo), tspan_h, xh_0, optionsh);          %
% ---------------------------------------------------------------------- %
%% Coast After Separation / phase 3 - Turn

tspan3 = [t2(end) t2(end)+100];
x3_0 = x2(end,:)';
x3_0(5) = Veiculo.m2_0;

options3 = options2;

[t3, x3] = ode45(@(t,x) ...
    movInclinadoCoast(t, x, Planeta, Veiculo), tspan3, x3_0, options3);

%% Second Burn / phase 4 - Turn

tspan4 = [t3(end) t3(end)+1000];
x4_0 = x3(end,:)';

options4 = odeset('Events', @(t, x) apogeuProjetado(t,x,Planeta,800), ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t4, x4, t_seco, x_seco, i_seco] = ode45(@(t,x) ...
    movInclinadoPower(t, x, Planeta, Veiculo, 2), tspan4, x4_0, options4);

%% Coast / phase 5 - Turn

tspan5 = [t_seco(end) t_seco(end)+10000];
x5_0 = x4(end,:)';

options5 = odeset('Events', @sensorApogeu, ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t5, x5, t_apogeu, x_apogeu, i_apogeu] = ode45(@(t, x) ...
    movInclinadoCoast(t, x, Planeta, Veiculo), tspan5, x5_0, options5);

%% Circularization / phase 6 - Turn

tspan6 = [t5(end) t5(end)+500];
x6_0 = x5(end,:)';

options6 = odeset('Events', @(t, x) perigeuProjetado(t, x, Planeta, 800), ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.1);

[t6, x6, t_circ, x_circ, i_circ] = ode45(@(t,x) ...
    movInclinadoPower(t, x, Planeta, Veiculo, 2), tspan6, x6_0, options6);

% ---------------------//-------------------- %

t = [t0; t1(2:end); t2(2:end); t3(2:end);
    t4(2:end); t5(2:end); t6(2:end,:)];
x = [x0; x1(2:end,:); x2(2:end,:); x3(2:end,:);
    x4(2:end,:); x5(2:end,:); x6(2:end,:)];

l = x(:,1);
h = x(:,2);
v = x(:,3);
gg = x(:,4);
m = x(:,5);

%% 4. RESULTADOS FINAIS 
imprimeEvento('Kick', t0(end), x0(end,:));
imprimeEvento('MECO', t1(end), x1(end,:));
imprimeEvento('Separação', t2(end), x2(end,:));
imprimeEvento('SEI', t3(end), x3(end,:));
imprimeEvento('SECO', t4(end), x4(end,:));
imprimeEvento('Apogeu', t5(end), x5(end,:));
imprimeEvento('Fim da Circularização', t6(end), x6(end,:));

% if ~isempty(t_coast)
%     for k = 1:length(i_coast)
%         if i_coast(k) == 1
%             imprimeEvento('Apogeu', t_coast(k), x_coast(k,:));
%         elseif i_coast(k) == 2
%             imprimeEvento('Perigeu', t_coast(k), x_coast(k,:));
%         elseif i_coast(k) == 3
%             imprimeEvento('Impacto (5km)', t_coast(k), x_coast(k,:));
%         end
%     end
% end

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



