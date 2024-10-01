clc
clear 
close all


%% =============================================================================
% Carrega as tabelas de referências da Petrobras para as proteções e para
% buscar condiçoes iniciais em pontos de operação reais
CarregaTabelas; 

%TabelaLimitesDinamicos=readtable('TabelaLimitesDinamicos.xlsx');    % Tabela completa com pré-cálculos dos limites dinâmicos em função da frequência

%TabSimulador = TabSimulador(:,1:3);

%MatrizVazao = table2array(TabelaLimitesDinamicos(:, [1,11,13])); %cortando a coluna LIMITES
MatrizVazao = table2array(TabSimulador(:,1:3));

import casadi.*
Freq_sym = MX.sym('F',1);
Press_sym = MX.sym('P',1);
%MatrizLimitesDinamicos_sym = MX.sym('M', size(MatrizVazao));
freq = 44;
pmon = 38.5;

%testa sel ponto
tic
Pontos = selPontos_casadi_vazao(Freq_sym, Press_sym, MatrizVazao);
f = Function('f', {Freq_sym, Press_sym}, {Pontos.FreqBCSS, Pontos.PressChegada, Pontos.VazaoOleo});
[FreqBCSS, PressChegada, VazaoOleo] = f(freq, pmon);
disp(FreqBCSS);
disp(PressChegada);
disp(VazaoOleo)
toc

tic
Pontos = selPontos_casadi_vazao_v2(Freq_sym, Press_sym, MatrizVazao);
f = Function('f', {Freq_sym, Press_sym}, {Pontos.FreqBCSS, Pontos.PressChegada, Pontos.VazaoOleo});
[FreqBCSS, PressChegada, VazaoOleo] = f(freq, pmon);
disp(FreqBCSS);
disp(PressChegada);
disp(VazaoOleo);
toc

Pontos=selPontos(freq,pmon,TabSimulador);
disp(Pontos)

tic
vazao_casadi_symb=Interpola_casadi_vazao(Freq_sym, Press_sym, MatrizVazao);
f = Function('f', {Freq_sym, Press_sym}, {vazao_casadi_symb});
vasao_casadi = full(f(freq, pmon));
disp(vasao_casadi);
toc

tic
vazao_casadi_symb=Interpola_casadi_vazao_v2(Freq_sym, Press_sym, MatrizVazao);
f = Function('f', {Freq_sym, Press_sym}, {vazao_casadi_symb});
vasao_casadi = full(f(freq, pmon));
disp(vasao_casadi);
toc

tic
vasao = Interpola(freq, pmon, TabSimulador, 3);
disp(vasao)
toc
% dif_vazoes=[];
% 
% cont = 1;
% for  freq = 40:0.5:60
% disp(cont)
% vazao_casadi = full(f(freq, pmon));
% vazao = Interpola(freq, pmon, TabSimulador, 3);
% dif = vazao-vazao_casadi;
% disp(dif)
% dif_vazoes(cont) = dif;
% cont = cont +1;
% 
% end

