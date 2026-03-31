h_vetor = (200:5:300);           
gg_vetor = (89:0.1:89.4);      
t_coast_vetor = (5:1:15);

% Conversões
deg2rad = pi/180;

% Constantes Planeta
Planeta.Re = 6371008.8;                                         % Raio médio da Terra
Planeta.g0 = 9.80665;                                           % Aceleração gravítica (nível no mar)
Planeta.h0 = 0;                                                 % Altitude relativa

% Estados Iniciais Veiculo
Veiculo.d = 1.5;                                                % Diâmetro do Veiculo
Veiculo.Aref = pi * (Veiculo.d/2)^2;                            % Área de Referência
Veiculo.CD = 0.2;                                               % Coeficiente de arrasto utopico

% Estágio 1
Veiculo.m1_0 = 35093.2;                                         % Massa Húmida - 1º estágio
Veiculo.mf1 = 6031.9;                                           % Massa final do primeiro estágio
Veiculo.Isp1 = 302.3333;                                        % Impulso específico (motor principal)
Veiculo.T2W1 = 1.4;                                             % Thrust to Weight ratio nominal
Veiculo.T1 = Veiculo.m1_0*Planeta.g0*Veiculo.T2W1;              % Thrust (motor principal)
Veiculo.m_flux1 = Veiculo.T1 / (Veiculo.Isp1 * Planeta.g0);     % Caudal mássico de prop

% Estágio 2
Veiculo.m2_0 = 3616.0;                                          % Massa húmida - 2º estágio
Veiculo.mf2 = 711.0;                                            % Massa final do segundo estágio
Veiculo.Isp2 = 327.3333;                                        % Impulos específico (motor secundário)
Veiculo.T2W2 = 0.7;                                             % Thrust to Weight ratio nominal
Veiculo.T2 = Veiculo.m2_0*Planeta.g0*Veiculo.T2W2;              % Thrust (motor secundário)
Veiculo.m_flux2 = Veiculo.T2 / (Veiculo.Isp2 * Planeta.g0);     % Caudal mássico de prop

Mapa_massas = zeros(length(h_vetor), length(gg_vetor), length(t_coast_vetor));
total_sims = numel(Mapa_massas);
fprintf('A inicializar ARES: Varredura Paramétrica 3D (%d combinações)...\n', total_sims);

for i = 1:length(h_vetor)
    for j = 1:length(gg_vetor)
        for k = 1:length(t_coast_vetor)

            % Injetar parametros no Veiculo
            Veiculo.h_kick = h_vetor(i);
            Veiculo.gg0 = gg_vetor(j) * (pi/180); % Converter para rad
            t_sei = t_coast_vetor(k);
            deg2rad = pi/180;

            % --- EXECUTAR SIMULACAO (Fases 0 a 6) ---
            try
                %% Power / phase 0 - Vertical

                x0_0 = [0; 0; 0.01; Veiculo.gg0; Veiculo.m1_0];
                tspan0 = [0 100];
                options0 = odeset('Events', @(t,x) kick(t, x, Veiculo.h_kick), ...
                    'RelTol', 1e-6, ...
                    'AbsTol', 1e-9, ...
                    'MaxStep', 0.1);

                [t0, x0, t_kick, ~, ~] = ode45(@(t,x) ...
                    movVerticalPower(t, x, Planeta, Veiculo), tspan0, x0_0, options0);
                if x0(end,2) < 1, error('Chão - 0'); end

                %% Power / phase 1 - Turn
                fase_atual = 1;

                tspan1 = [t_kick(end) t_kick(end)+200];
                x1_0 = x0(end, :)';

                options1 = odeset('Events', @(t, x) sensorCombustivel(t,x,Veiculo.mf1), ...
                    'RelTol', 1e-6, ...
                    'AbsTol', 1e-9, ...
                    'MaxStep', 0.1);

                [t1, x1, t_meco, ~, ~] = ode45(@(t, x) ...
                    movInclinadoPower(t, x, Planeta, Veiculo, 1), tspan1, x1_0, options1);
                if x1(end,2) < 1, error('Chão - 1'); end

                %% Coast Until Separation / phase 2 - Turn
                fase_atual = 2;

                tspan2 = [t_meco(end) t_meco(end)+10];
                x2_0 = x1(end,:)';

                options2 = odeset('Events', @sensorChao, ...
                    'RelTol', 1e-6, ...
                    'AbsTol', 1e-9, ...
                    'MaxStep', 0.1);

                [t2, x2] = ode45(@(t, x) ...
                    movInclinadoCoast(t, x, Planeta, Veiculo), tspan2, x2_0, options2);
                if x2(end,2) < 1, error('Chão - 2'); end

                %% Coast After Separation / phase 3 - Turn
                fase_atual = 3;
                % AQUI INJETAMOS O TEMPO DE COAST VARIÁVEL
                tspan3 = [t2(end) t2(end) + t_sei];
                x3_0 = x2(end,:)';
                x3_0(5) = Veiculo.m2_0;
                options3 = options2;
                [t3, x3] = ode45(@(t,x) ...
                    movInclinadoCoast(t, x, Planeta, Veiculo), tspan3, x3_0, options3);
                if x3(end,2) < 1, error('Chão - 3'); end

                %% Second Burn / phase 4 - Turn
                fase_atual = 4;

                tspan4 = [t3(end) t3(end)+1000];
                x4_0 = x3(end,:)';

                options4 = odeset('Events', @(t, x) apogeuProjetado(t,x,Planeta,800), ...
                    'RelTol', 1e-6, ...
                    'AbsTol', 1e-9, ...
                    'MaxStep', 0.1);

                [t4, x4, t_seco, ~, ~] = ode45(@(t,x) ...
                    movInclinadoPower(t, x, Planeta, Veiculo, 2), tspan4, x4_0, options4);
                if x4(end,2) < 1, error('Chão - 4'); end

                %% Coast to Apogeu / phase 5 - Turn
                fase_atual = 5;

                tspan5 = [t_seco(end) t_seco(end)+10000];
                x5_0 = x4(end,:)';

                options5 = odeset('Events', @sensorApogeu, ...
                    'RelTol', 1e-6, ...
                    'AbsTol', 1e-9, ...
                    'MaxStep', 0.1);

                [t5, x5, ~, ~, ~] = ode45(@(t, x) ...
                    movInclinadoCoast(t, x, Planeta, Veiculo), tspan5, x5_0, options5);
                if x5(end,2) < 1, error('Chão - 5'); end

                %% Circularization / phase 6 - Turn
                fase_atual = 6;

                tspan6 = [t5(end) t5(end)+500];
                x6_0 = x5(end,:)';

                options6 = odeset('Events', @(t, x) perigeuProjetado(t, x, Planeta, 800), ...
                    'RelTol', 1e-6, ...
                    'AbsTol', 1e-9, ...
                    'MaxStep', 0.1);

                [t6, x6, ~, ~, ~] = ode45(@(t,x) ...
                    movInclinadoPower(t, x, Planeta, Veiculo, 2), tspan6, x6_0, options6);
                if x6(end,2) < 1, error('Chão - 6'); end

                m_final = x6(end, 5);
                if m_final > Veiculo.mf2
                    Mapa_massas(i,j,k) = m_final - Veiculo.mf2;
                else
                    Mapa_massas(i,j,k) = 0;
                end
            catch
                Mapa_massas(i,j,k) = 0; % Erro ou colisão
            end
        end
    end
    fprintf('Progresso ARES: %.1f%%\n', (i/length(h_vetor))*100);
end


[max_sobra, idx] = max(Mapa_massas(:)); % Encontra o máximo no cubo 3D

% ind2sub agora devolve 3 índices
[i_opt, j_opt, k_opt] = ind2sub(size(Mapa_massas), idx); 

h_optimo = h_vetor(i_opt);
gg_optimo = gg_vetor(j_opt);
t_sei_optimo = t_coast_vetor(k_opt);

fprintf('\n================ SELEÇÃO ARES 3D ================ \n');
if max_sobra > 0
    fprintf('Combinação Ótima Encontrada!\n');
    fprintf('Altitude de Kick (h): %.0f m\n', h_optimo);
    fprintf('Ângulo de Kick (gg): %.2f deg\n', gg_optimo);
    fprintf('Tempo de Coast (Fase 3): %.1f s\n', t_sei_optimo);
    fprintf('Massa de Sobra (Payload real): %.2f kg\n', max_sobra);
else
    fprintf('Atenção: Nenhuma combinação atingiu os 800km com sucesso.\n');
end
fprintf('================================================= \n');

%% Visualização dos Resultados (Scatter 3D)
% Para mapear um cubo 3D, mostramos apenas os pontos onde houve sucesso
[X, Y, Z] = meshgrid(gg_vetor, h_vetor, t_coast_vetor);

% Filtra apenas os índices onde a massa é maior que zero (sucesso)
indices_sucesso = find(Mapa_massas > 0);

if ~isempty(indices_sucesso)
    figure('Name', 'Otimização de Trajetória ARES - Espaço 3D', 'Color', 'w');
    % scatter3(X, Y, Z, tamanho_do_ponto, cor)
    scatter3(X(indices_sucesso), Y(indices_sucesso), Z(indices_sucesso), ...
             50, Mapa_massas(indices_sucesso), 'filled', 'MarkerEdgeColor', 'k');
         
    cb = colorbar;
    ylabel(cb, 'Massa de Sobra (kg)');
    colormap(jet); % Usa um mapa de cores vibrante
    
    xlabel('Ângulo de Kick (deg)');
    ylabel('Altitude de Kick (m)');
    zlabel('Tempo de Coast Pós-Separação (s)');
    title('Nuvem de Soluções Ótimas ARES');
    grid on;
    view(45, 30); % Roda a câmara para uma boa perspetiva 3D
end