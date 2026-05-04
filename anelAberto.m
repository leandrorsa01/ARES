linearizacao

polos_instaveis = zeros(1, length(pontos_nominais));
mach_voo = zeros(1, length(pontos_nominais));

fprintf('\n--- Análise Modal ---\n');

for i = 1:length(pontos_nominais)
    A_atual = A_matrizes{i};
    mach_voo(i) = pontos_nominais{i}.mach;
    
    % Calcular os valores próprios (polos) da matriz A
    polos = eig(A_atual);
    
    % Encontrar o polo com a maior parte real (o mais instável)
    max_real = max(real(polos));
    polos_instaveis(i) = max_real;
    
    % Imprimir o veredicto no ecrã
    if max_real > 1e-4
        estado = 'INSTÁVEL';
    elseif max_real < -1e-4
        estado = 'ESTÁVEL';
    else
        estado = 'NEUTRO';
    end
    
    fprintf('[%s] Polo Máximo: %8.4f -> %s\n', pad(nomes{i}, 8), max_real, estado);
end

% Desenhar o gráfico da instabilidade
figure('Name', 'Instabilidade do Foguetão em Anel Aberto');
plot(mach_voo, polos_instaveis, '-ro', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
grid on;
title('Evolução da Instabilidade (Parte Real Máxima dos Polos)');
xlabel('Número de Mach');
ylabel('Parte Real Máxima [rad/s] (Valores Positivos = Instável)');
yline(0, 'k--', 'Linha de Estabilidade', 'LineWidth', 1.5);