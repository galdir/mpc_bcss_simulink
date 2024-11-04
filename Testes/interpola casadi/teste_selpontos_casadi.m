clc
clear
close all


%% =============================================================================
% Carrega as tabelas de referências da Petrobras para as proteções e para
% buscar condiçoes iniciais em pontos de operação reais
CarregaTabelas;


MatrizSimulador = table2array(TabSimulador);
MatrizVazao = table2array(TabSimulador(:,1:3));

import casadi.*
Freq_sym = MX.sym('F',1);
Press_sym = MX.sym('P',1);
tic
selPontos_casadi_vazao_sym = selPontos_casadi_vazao(Freq_sym, Press_sym, MatrizVazao);
f_selPontos_casadi_vazao_sym = Function('f', {Freq_sym, Press_sym}, {selPontos_casadi_vazao_sym.FreqBCSS, selPontos_casadi_vazao_sym.PressChegada, selPontos_casadi_vazao_sym.VazaoOleo});
toc

freq = 44;
pmon = 38;


disp('testa selpontos_casadi')
tic
[FreqBCSS, PressChegada, VazaoOleo] = f_selPontos_casadi_vazao_sym(freq, pmon);
disp(FreqBCSS);
disp(PressChegada);
disp(VazaoOleo)
toc

disp('testa selpontos original')
pontos=selPontos(freq,pmon,MatrizVazao);
disp(pontos)

disp('testando frequencias baixas')
freq = 1;
pmon = 1;

disp('testa selpontos_casadi')
tic
[FreqBCSS, PressChegada, VazaoOleo] = f_selPontos_casadi_vazao_sym(freq, pmon);
disp(FreqBCSS);
disp(PressChegada);
disp(VazaoOleo)
toc

disp('testa selpontos original')
pontos=selPontos(freq,pmon,MatrizVazao);
disp(pontos)


disp('testando frequencias altas')
freq = 100;
pmon = 100;

disp('testa selpontos_casadi')
tic
[FreqBCSS, PressChegada, VazaoOleo] = f_selPontos_casadi_vazao_sym(freq, pmon);
disp(FreqBCSS);
disp(PressChegada);
disp(VazaoOleo)
toc

disp('testa selpontos original')
pontos=selPontos(freq,pmon,MatrizVazao);
disp(pontos)
