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

Pos0 = [0; 0; Planeta.h0];
Quat0 = [1; 0; 0; 0];
Vel0 = [0; 0; 0];
Rot0 = [0; 0; 0];
Mflux0 = [Veiculo.m_RP1_S1; Veiculo.m_LOX_S1];

x0_0 = [Pos0; Quat0; Vel0; Rot0; Mflux0];