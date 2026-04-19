% --------------------------------------------------------------------- %
%                  OTIMIZADOR DE TRAJETÓRIA ARES 3D                     %
% --------------------------------------------------------------------- %
close all; clear; clc;

h_vetor = (150:1:500);           
gg_vetor = (88.5:0.1:89.9);      
t_coast_vetor = (2);

deg2rad = pi/180;
Planeta.Re = 6371008.8;                                         
Planeta.g0 = 9.80665;                                           
Planeta.h0 = 0;                                                 
Veiculo = loadVeiculo(1);

Mapa_massas = zeros(length(h_vetor), length(gg_vetor), length(t_coast_vetor));
total_sims = numel(Mapa_massas);
fprintf('A inicializar ARES: Varredura Paramétrica 3D (%d combinações)...\n', total_sims);

start_time = tic;

for i = 1:length(h_vetor)
    for j = 1:length(gg_vetor)
        for k = 1:length(t_coast_vetor)
                        
            Veiculo.h_pitchOver = h_vetor(i);
            Veiculo.gg0 = gg_vetor(j) * deg2rad; 
            t_sei = t_coast_vetor(k);
            
            % --- EXECUTAR SIMULAÇÃO ---
            try
                % Phase 0 - Vertical
                x0_0 = [0; 0; 0.01; Veiculo.gg0; Veiculo.m1_0];
                ev0 = @(t,x) superEvento(t, x, Planeta, 'pitchOver', Veiculo.h_pitchOver);
                [~, x0] = executarFase(@(t,x) movVerticalPower(t, x, Planeta, Veiculo), 0, 100, x0_0, ev0);
                if x0(end,2) < 1, error('Chão - 0'); end
                
                % Phase 1 - Turn
                ev1 = @(t, x) superEvento(t,x,Planeta, 'sensorCombustivel', Veiculo.mf1);
                [~, x1] = executarFase(@(t, x) movInclinadoPower(t, x, Planeta, Veiculo, 1), 0, 200, x0(end,:)', ev1);
                if x1(end,2) < 1, error('Chão - 1'); end
                
                % Phase 2 - Coast Until Separation
                [~, x2] = executarFase(@(t, x) movInclinadoCoast(t, x, Planeta, Veiculo), 0, 10, x1(end,:)', []);
                if x2(end,2) < 1, error('Chão - 2'); end
                
                % Phase 3 - Coast After Separation 
                x3_0 = x2(end,:)'; x3_0(5) = Veiculo.m2_0;
                [~, x3] = executarFase(@(t,x) movInclinadoCoast(t, x, Planeta, Veiculo), 0, t_sei, x3_0, []);
                if x3(end,2) < 1, error('Chão - 3'); end
                
                % Phase 4 - Second Burn
                ev4 = @(t, x) superEvento(t,x,Planeta,'apogeuProjetado',800);
                [~, x4] = executarFase(@(t,x) movInclinadoPower(t, x, Planeta, Veiculo, 2), 0, 1000, x3(end,:)', ev4);
                if x4(end,2) < 1, error('Chão - 4'); end
                
                % Phase 5 - Coast to Apogeu
                ev5 = @(t, x) superEvento(t,x,Planeta,'sensorApogeu',0);
                [~, x5] = executarFase(@(t, x) movInclinadoCoast(t, x, Planeta, Veiculo), 0, 10000, x4(end,:)', ev5);
                if x5(end,2) < 1, error('Chão - 5'); end
                
                % Phase 6 - Circularization
                ev6 = @(t, x) superEvento(t, x, Planeta, 'perigeuProjetado', 800);
                [~, x6] = executarFase(@(t,x) movInclinadoPower(t, x, Planeta, Veiculo, 2), 0, 500, x5(end,:)', ev6);
                if x6(end,2) < 1, error('Chão - 6'); end
                
                % Cálculo do delta de massa final
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
    elapsed_time = toc(start_time);                 % Tempo que já passou (segundos)
    progresso_frac = i / length(h_vetor);           % Fração concluída (ex: 0.01 para 1%)
    
    tempo_total_est = elapsed_time / progresso_frac; % Estimativa do tempo total
    tempo_restante = tempo_total_est - elapsed_time; % Tempo que falta
    
    % Converter segundos em Horas:Minutos:Segundos
    h_rest = floor(tempo_restante / 3600);
    m_rest = floor(mod(tempo_restante, 3600) / 60);
    s_rest = floor(mod(tempo_restante, 60));
    
    fprintf('Progresso ARES: %5.1f%% | Tempo Restante Est.: %02d:%02d:%02d\n', ...
            progresso_frac * 100, h_rest, m_rest, s_rest);
end

% --- EXTRAÇÃO DE RESULTADOS E PLOTS ---
[max_sobra, idx] = max(Mapa_massas(:)); 
[i_opt, j_opt, k_opt] = ind2sub(size(Mapa_massas), idx); 

h_optimo = h_vetor(i_opt);
gg_optimo = gg_vetor(j_opt);
t_sei_optimo = t_coast_vetor(k_opt);

fprintf('\n================ SELEÇÃO ARES 3D ================ \n');
if max_sobra > 0
    fprintf('Combinação Ótima Encontrada!\n');
    fprintf('Altitude de Pitch-Over (h): %.0f m\n', h_optimo);
    fprintf('Ângulo de Pitch-Over (gg): %.2f deg\n', gg_optimo);
    fprintf('Tempo de Coast (Fase 3): %.1f s\n', t_sei_optimo);
    fprintf('Massa de Sobra (Payload real): %.2f kg\n', max_sobra);
else
    fprintf('Atenção: Nenhuma combinação atingiu os 800km com sucesso.\n');
end
fprintf('================================================= \n');

%% Visualização dos Resultados (Scatter 3D)
[X, Y, Z] = meshgrid(gg_vetor, h_vetor, t_coast_vetor);
indices_sucesso = find(Mapa_massas > 0);

if ~isempty(indices_sucesso)
    figure('Name', 'Otimização de Trajetória ARES - Espaço 3D', 'Color', 'w');
    scatter3(X(indices_sucesso), Y(indices_sucesso), Z(indices_sucesso), ...
             50, Mapa_massas(indices_sucesso), 'filled', 'MarkerEdgeColor', 'k');
         
    cb = colorbar;
    ylabel(cb, 'Massa de Sobra (kg)');
    colormap(jet); 
    
    xlabel('Ângulo de Pitch-Over (deg)');
    ylabel('Altitude de Pitch-Over (m)');
    zlabel('Tempo de Coast Pós-Separação (s)');
    title('Nuvem de Soluções Ótimas ARES');
    grid on;
    view(45, 30); 
end