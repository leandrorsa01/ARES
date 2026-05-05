linearizacao
              
max_vel      = 50;         
max_taxas    = deg2rad(5); 
max_angulo = deg2rad(2);
max_quat_vetorial = sin(max_angulo / 2);

Q = diag([
    1/max_quat_vetorial^2;   % 6: q2 (Pitch)
    1/max_quat_vetorial^2;   % 7: q3 (Yaw)
    1000/max_vel^2;             % 9: v
    1000/max_vel^2;             % 10: w
    1/max_taxas^2;           % 12: q (Pitch rate)
    1/max_taxas^2;           % 13: r (Yaw rate)
    ]);

max_gimbal = deg2rad(5);

R = diag([
    .01/max_gimbal^2;        % 1: dy (Gimbal Pitch)
    .01/max_gimbal^2;        % 2: dz (Gimbal Yaw)
]);

K_matrizes     = cell(1, length(pontos_nominais));
A_red_matrizes = cell(1, length(pontos_nominais));
B_red_matrizes = cell(1, length(pontos_nominais));
Acl_matrizes   = cell(1, length(pontos_nominais));
E_matrizes     = cell(1, length(pontos_nominais));
estados_lqr    = [6, 7, 9, 10, 12, 13];
inputs_lqr     = [1, 2];

for i = 1:length(pontos_nominais)

    A_red = A_matrizes{i}(estados_lqr, estados_lqr);
    B_red = B_matrizes{i}(estados_lqr, inputs_lqr);

    A_red_matrizes{i} = A_red;
    B_red_matrizes{i} = B_red;
    
    [K, S, E] = lqr(A_red, B_red, Q, R);
    
    K_matrizes{i} = K;
    E_matrizes{i} = E;

    Acl_matrizes{i} = A_red - B_red * K;
    
    fprintf('[%s] Polo em Anel Fechado (max real): %8.4f\n', ...
            pad(nomes{i}, 8), max(real(E)));
end

evento = 8;

fprintf('\n--- Análise Clássica de Amortecimento e Frequência (Ponto: %s) ---\n', nomes{evento});
damp(Acl_matrizes{evento});