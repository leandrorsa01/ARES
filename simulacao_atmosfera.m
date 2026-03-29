%% SCRIPT DE DADOS: 
clear; clc;

% 1. Definir o vetor de altitudes metro a metro 
h_metro_a_metro = (0:1:100000)'; 

% 2. Chamar a função 
fprintf('A calcular modelo ISA para o Nova-C... ');
[rho_total, P_total, T_total, a_total] = atmosfera_100km(h_metro_a_metro);


% 3. Verificar o tamanho dos dados
disp(['Pontos calculados: ', num2str(length(rho_total))]);


% Criar uma tabela 
Dados_Atmosfera = table(h_metro_a_metro, T_total, P_total, rho_total, a_total, ...
    'VariableNames', {'Altitude_m', 'Temp_K', 'Pressao_Pa', 'Densidade_kgm3', 'VelSom_ms'});

