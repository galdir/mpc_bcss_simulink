clc
clear all
close all

% =============================================================================
% Necessário ajustar o Path do Matlab para apontar onde estão as tabelas e dados Petrobras e arquivos de uso geral
% Na pasta Uteis, há uma rotina AjustaPath.m que pode ser copiada para a raiz do
% repositório e ajustada para a necessidade de cada um. Esse arquivo na
% raiz do repositório não será enviado para o github
% =============================================================================



%% =============================================================================
% Carrega as tabelas de referências da Petrobras para as proteções e para
% buscar condiçoes iniciais em pontos de operação reais
CarregaTabelas; 


%% =============================================================================
% Condição inicial  das variáveis do processo e das entradas     PSuc [bar],    PChegada [bar]        Freq [Hz]
%  [YIni,UIni]=SelCondicaoInicial('2024-07-12 10:00:00');          % PSuc=77.4    PChegada=31.4      Freq = 54.9Hz
% [YIni,UIni]=SelCondicaoInicial('2024-07-12 15:45:00');          % PSuc=78.9    PChegada=34.2      Freq = 53.9Hz
 [YIni,UIni]=SelCondicaoInicial('2024-07-17 00:00:00');               % PSuc=97.4    PChegada=35.3      Freq = 40Hz      Rampa de aceleração
%[YIni,UIni]=SelCondicaoInicial('2024-07-17 01:00:00');               %Rampa de aceleração

% A condição inicial da vazão precisamos apenas para inicializar a visualização dos mapas  
VazaoIni=Interpola(UIni(1),YIni(2)*1.019716,TabSimulador,3);  % Entra Freq [Hz] e Presão [Kgf/cm2] para retornar a vazão em m3/dia

%% =============================================================================
% Carrega tabela com plano de experimentos programados para mudanças
% automáticas de Freq. e PMonAlvo para simulação
% Plano=readtable('PlanoTesteCampo.xlsx');    % Plano que reproduz testes de campo no dia 12/07/2024, neste caso, usamos a inicialização do dia 12/07 às 10h

Plano=readtable('PlanoConstante.xlsx');     % Plano com valor constante pré-estabelecido. Neste caso, usamos a inicialização do dia 12/07 às 10h
% Plano=readtable('PlanoDegrauFreq.xlsx');  % Plano com valor constante pré-estabelecido. Neste caso, usamos a inicialização do dia 12/07 às 10h
% Plano=readtable('PlanoDegrauPMon.xlsx');  % Plano com valor constante pré-estabelecido. Neste caso, usamos a inicialização do dia 12/07 às 10h

StopTime=Plano.Tempo(end);                          % O tempo de simulação segue o plano definido na tabela 

% Inicializar com o primeiro registro do plano proposto
FreqAlvoIni=Plano.Frequencia(1);                    % Resgata da tabela o ponto de inicial da Frequencia [Hz]
PMonAlvoIni=Plano.PMonAlvo(1);                    % Resgata da tabela o ponto de inicial da PMonAlvo [bar]
% FreqAlvoIni=50;
% PMonAlvoIni=30;


%% =============================================================================
% Carrega uma rede (ESN qualquer) para ser usada como modelo do processo
% Rede_Processo = load('.\ESN\weightsESNx_JUB27n100.mat');
Rede_Processo = load('..\Modelos\ESN\weightsESNx_TR300_TVaz0.8_RaioE0.1.mat');
%Rede_Processo = load('..\Modelos\\ESN\weightsESNx_TR400_TVaz0.9_RaioE0.4.mat');
%Rede_Processo = load('.\ESN\weightsESNx_TR900_TVaz0.9_RaioE0.4.mat');


% Vale a pena inserir ruido na saida do processo para simular mundo real e avaliar robustez do controlador
NivelRuido=0;       % Define nivel de ruido aditivo (0 a 10) para as saidas do processo

%% =============================================================================
% Escolha o modelo Preditor no MPC
TipoRede=1;        % 1=ESN ou  2=LSTM 

% Carrega rede ESN que será utilizada como preditor no MPC
% ESN_MPC = load('.\ESN\weightsESNx_JUB27n100.mat');
% ESN_MPC = load('..\Modelos\ESN\weightsESNx_TR300_TVaz0.8_RaioE0.1.mat');
ESN_MPC = load('..\Modelos\ESN\weightsESNx_TR400_TVaz0.9_RaioE0.4.mat');
ESN_MPC.data.tipo = TipoRede;
ModeloPreditor = ESN_MPC;

% ==================
% Carrega o modelo LSTM
% pasta_modelos = '.\modelos_lstm2_casadi\';
% LSTM_MPC = carregaModelosCasadi(pasta_modelos);
% % Falta inserir o TIPO = 2
% ModeloPreditor = LSTM_MPC;

%% =============================================================================
% Definição da base de tempo para simulação. 
% OBS: Simulação funciona com Ts diferente de 10s, mas isso faz sair da escala de tempo real e também faz errar o cálculo
% do volume produzido, já que a vazão estimada é em m3/dia. Assim, é importante manter Ts = 10
Ts =10;                                            % Passo de simulação (1 passo = 10s do processo) 
PassoMPC =3;                              % Proporção de amostras para atuação do MPC (3 passos = 30s do processo)

%% =============================================================================
% Define configurações do MPC

%% Restrições máximas e mínimas para as variáveis manipuladas (entradas do processo)
FreqMaxMin=[60; 40];
PMonAlvoMaxMin=[40; 25];
umax  = [FreqMaxMin(1); PMonAlvoMaxMin(1)];       % Valor máximo das manipuladas (Freq e PMonAlvo)
umin  =  [FreqMaxMin(2); PMonAlvoMaxMin(2)] ;      % Valor mínimo das manipuladas  (Freq e PMonAlvo)
 
dumax = [0.5 ; 1];                                                          %Variação máxima nas manipuladas [ Hz    bar ]
dumin = [0; 0];                                                               %Variação mínima nas manipuladas [ Hz    bar ]

%% ======================
% Parâmetros do Controlador
Hp =  10;                     % Horizonte de predição
Hc =  4 ;                      % Horizonte de controle
Qy=  [2 1];                   % Qy - Peso das Controladas (PSuc e PChegada) prioridade da variável controlada voltar para respectiva faixa (calculada pelas proteções dinâmicas)
R=    [1 1];                   % R - Peso na variação da ação de controle - Delta U em (Freq. e PMonAlvo) 
Qu = [1 1];                   % Qu - Peso dos Alvos Desejados (Freq. e PMonAlvo). Ponderação priorização dos alvos (ponto do mapa)

% Inicializa estados para atuação do MPC
MatrizYIni= repmat(YIni,1,Hp+1);                            % Condição inicial das variáveis (preenche horizonte até Hp+1 com valores iniciais)
DeltaUIni= repmat(dumax,Hc,1);                             % Condição incial dos deltaU para todo o horizonte de controle
InicializaMPC=[MatrizYIni(:);DeltaUIni;YIni(1:2)];   % Monta vetor de estados iniciais para operação do MPC
% YIni são as condições iniciais das 10 variáveis do processo
% DeltaUIni são as varições para cada variável manipulada
% YIni(1:2), São a PSuc e a PChegada, ou seja, é a condição inicial das variáveis controladas

%% =============================================================================
disp('Configurações do usuário e parâmetros do controlador foram carregados para a área de trabalho')

disp("Python acessível no Matlab?:");
disp(pyenv);



