clc
%===============================
% Carrega tabelas XLS para variáveis
TabRestricoesDinamicas=readtable('DP.xlsx');              % Tabela de valores de referência para as variáveis
FxPercent=readtable('DP-Faixas.xlsx');                           % Tabela das faixas percentuais para disparar os alarmes LL, L, H e HH
BTP=readtable('DoBTP.xlsx');                                           % Tabela de dados do teste de produção
ProtecaoFixa =readtable('FixedProtections.xlsx');            % Carrega tabelas de proteção fixa da Petrobras
TabSimulador=LeConverteNomes('DoSimulador.xlsx');  % Tabela do simulador (Análise de Sensibilidade)
TabelaLimitesDinamicos=readtable('TabelaLimitesDinamicos.xlsx');    % Tabela completa com pré-cálculos dos limites dinâmicos em função da frequência
%===============================
% Avisa na tela
disp('Tabelas Petrobras carregadas para a área de trabalho')         %Carrega tabelas de proteção fixa da Petrobras

