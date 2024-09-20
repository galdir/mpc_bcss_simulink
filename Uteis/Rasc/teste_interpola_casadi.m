clc
clear all
close all


%% =============================================================================
% Carrega as tabelas de referências da Petrobras para as proteções e para
% buscar condiçoes iniciais em pontos de operação reais
%CarregaTabelas; 

TabSimulador=LeConverteNomes('DoSimulador.xlsx');

import casadi.*
P = SX.sym('P',2);
vazao_casadi_symb=Interpola_casadi(P(1),P(2)*1.019716,TabSimulador,3);
disp(vazao_casadi_symb)

% teste com valor real
freq = 45;
pmon_alvo = 35;
vasao_casadi = Interpola_casadi_real(freq, pmon_alvo, TabSimulador, 3);
disp(vasao_casadi);

vasao = Interpola(freq, pmon_alvo, TabSimulador, 3);
disp(vasao)
