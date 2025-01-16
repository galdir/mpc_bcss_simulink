% Carrega para a área de trabalho dados e parâmetros gerais para atuação do controlador MPC

% Começa carregando dados de uso geral e comum a qualquer ambiente (MPC ou CBR)
CarregaDados;                % Função para carregar tabelas Petrobras e dados gerais necessários para a simulação

WallTime=9;     % Tempo limite (em segundos) para o cálculo do Solver

%% =============================================================================
% Escolha o modelo Preditor no MPC
TipoRede=1;        % 1=ESN ou  2=LSTM 

% Carrega rede ESN que será utilizada como preditor no MPC
% NomeESN= 'weightsESNx_TR300_TVaz0.8_RaioE0.1.mat';    % Esta foi usada como processo
NomeESN='weightsESNx_TR400_TVaz0.9_RaioE0.4.mat'; %usado por leizer
%NomeESN='weightsESNx_TR100_TVaz0.12_RaioE0.99_mape0.55.mat'; %criada no matlab
ESN_MPC = load(NomeESN);
ESN_MPC.data.tipo = TipoRede;     % Insere o tipo de rede na estrutura do modelo
ModeloPreditor = ESN_MPC;

%% =============================================================================
% Define configurações do MPC
PassoMPC =3;                              % Proporção de amostras para atuação do Controlador (3 passos = 30s do processo)

%% ======================
% Parâmetros do Controlador (ainda por definir a melhor sintonia)
Hp = 10;                               % Horizonte de predição
Hc = Hp-1;                         % Horizonte de controle
Qy=  0*diag([1  10]);              % Qy - Peso das saidas controladas por setpoint = PChegada e Vazao)
Qu = 1*diag([10  1]);              % Qu - Peso das ações de controle nas entradas (Alvos Desejados em  Freq. e PMonAlvo)
Qx= 0*diag(ones(1,11));    % Peso para os erros de estimação das  variáveis do processo
R=  0*diag([15  15]);             % R - Peso na variação das ações de controle - DeltaU em Freq. e PMonAlvo 

%% =============================================================================
% Considerando a diferença entre as grandezas, ajustamos os pesos para ter ua relação mais equilibrada
% na definição das matrizes de ponderação

%                     PSuc   PChegada    PDiff    PDescarga   Tmotor      ITorque    ITotal     TSuc     Vibração   TChegada    Vazao
LimitesMin=   [  0             10               0               0                 0               0              0            0              0                 0                 0    ];    
LimitesMax=  [ 250          65            250            250             200          200         250         200           4               200           1000    ];

% Ajusta matriz de pesos dos estados em função das respectivas grandezas
Peso =1./LimitesMax;    
Peso=diag(Peso);
Qx=Qx*Peso;      

% Ajusta matriz de pesos para as ações de controle em função das respectivas grandezas
%             Freq.                       PMonAlvo   (usa como referência os limtes máximos definidos)
Peso = [  1/umax(1)               1/umax(2)  ];     
Peso=diag(Peso);
Qu=Qu*Peso;                                            % Ajusta matriz de pesos em função das respectivas grandezas
R=R*Peso;                                                 % Ajusta matriz de pesos em função das respectivas grandezas

% Ajusta matriz de pesos das saidas em função das respectivas grandezas
%             PChegada                             Vazao
Peso = [  1/LimitesMax(2)          1/LimitesMax(11)  ];     
Peso=diag(Peso);
Qy=Qy*Peso;                                            % Ajusta matriz de pesos em função das grandezas

%% =============================================================================
disp('Configurações e parâmetros do controlador MPC foram carregados para a área de trabalho')
%% =============================================================================
% Fim da rotina principal

% Só para o Simulink saber das dimensões do demux na saida do CasadiBlock
% Isso será refeito internamente na inicialização do Casadi Block
nu=length(Qu);
ny=length(Qy);
nx=length(Qx);

