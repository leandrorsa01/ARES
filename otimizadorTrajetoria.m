h_vetor = (50:10:500);
gg_vetor = (85:0.5:89.5);

Mapa_massas = zeros(length(h_vetor),length(gg_vetor));

fprintf('A inicializar varredura paramétrica (%d combinações)...\n', numel(Mapa_massas));

for i = 1:length(h_vetor)
    for j = 1:length(gg_vetor)
        
        % Injetar parametros no Veiculo
        Veiculo.h_kick = h_vetor(i);
        Veiculo.gg0_kick = gg_vetor(j) * (pi/180); % Converter para rad
        
        % --- EXECUTAR SIMULACAO (Fases 0 a 4) ---
        % Aqui chamas a tua logica de ODE45 que ja tens
        try
            % [t, x] = ode45(...);
            
            % --- CRITERIO DE SUCESSO ---
            % Se chegou ao apogeu de 800km com combustivel de sobra
            if h_final >= 795000 && m_final > Veiculo.mf2
                Mapa_Massa(i,j) = m_final - Veiculo.mf2; % Sobra de massa
            else
                Mapa_Massa(i,j) = 0; % Falhou (caiu ou ficou sem gota)
            end
        catch
            Mapa_Massa(i,j) = 0; % Erro numerico (ex: singularidade)
        end
    end
    fprintf('Progresso: %.1f%%\n', (i/length(h_vetor))*100);
end