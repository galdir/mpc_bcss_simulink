% Para recriar todas as tabelas à serem consumidas pelo solver
% Deve er atualizada toda vez que recebermos novas tabelas DoBTP e DoSimulador
clc
clear all
close all

CriaTabelaSimuladorContas   % Deve ser a primeira para poder gerar as contas do simulador com maior resolução
CriaTabelaLimitesDinamicos   
CriaTabelaCurvasIsometricas % É a própria tabela do simulador mas com resolução específica adequada para o plot da curva isométrica
