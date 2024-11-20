
%===============================
% Carrega tabelas XLS para variáveis
TabRestricoesDinamicas=readtable('DP.xlsx');              % Tabela de valores de referência para as variáveis
MatrizRestricoesDinamicas=table2array(TabRestricoesDinamicas);   % Observar que existem variáveis desta tabela que nós não vamos usar

FxPercent=readtable('DP-Faixas.xlsx');            % Tabela das faixas percentuais para disparar os alarmes LL, L, H e HH
FxPercent=table2array(FxPercent(:,2:end));     % Extrai a primeira coluna com os simbolos dos alarmes
FxPercent=[ [0;0;0;0]    FxPercent ];                    % Preenche a primeira coluna com zeros para manter o mesmo indice da tabela DP

BTP=readtable('DoBTP.xlsx');                             % Tabela de dados do teste de produção
BTP=table2array(BTP);

ProtecaoFixa =readtable('FixedProtections.xlsx');            % Carrega tabelas de proteção fixa da Petrobras
ProtecaoFixa=table2array(ProtecaoFixa(:,2:end));           % Extrai a primeira coluna da tabela com simbolos de Max/Min

TabSimulador=LeConverteNomes('DoSimulador.xlsx');  % Tabela do simulador (Análise de Sensibilidade)
MatrizSimulador=table2array(TabSimulador);                   % Para os casos em que não poderemos usar tabelas, mas sim matrizes

MatrizSimuladorContas=readtable('TabSimuladorContas.xlsx');   % Tabela do simulador com maior resolulçao para cálculos de limites com maior precisão
MatrizSimuladorContas=table2array(MatrizSimuladorContas);

TabelaLimitesDinamicos=readtable('TabelaLimitesDinamicos.xlsx');    % Tabela completa com pré-cálculos dos limites dinâmicos em função da frequência
MatrizLimitesDinamicos=table2array(TabelaLimitesDinamicos);

TabelaIsometricas=readtable('TabelaCurvasIsometricas.xlsx');    % Tabela com resolução desejada para o plot das curvas isométricas nos mapas
MatrizIsometricas=table2array(TabelaIsometricas);
%===============================
% Avisa na tela
disp('Tabelas Petrobras carregadas para a área de trabalho')         %Carrega tabelas Petrobras

