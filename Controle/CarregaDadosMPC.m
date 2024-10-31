% Carrega para a área de trabalho dados e parâmetros gerais para atuação do controlador MPC

% Começa carregando dados de uso geral e comum a qualquer ambiente (MPC ou CBR)
CarregaDados;                % Função para carregar tabelas Petrobras e dados gerais necessários para a simulação

%% =============================================================================
% Escolha o modelo Preditor no MPC
TipoRede=1;        % 1=ESN ou  2=LSTM 

% Carrega rede ESN que será utilizada como preditor no MPC
% ESN_MPC = load('weightsESNx_JUB27n100.mat');
% ESN_MPC = load('weightsESNx_TR300_TVaz0.8_RaioE0.1.mat');    % Esta foi usada como processo
ESN_MPC = load('weightsESNx_TR400_TVaz0.9_RaioE0.4.mat');
ESN_MPC.data.tipo = TipoRede;     % Insere o tipo de rede na estrutura do modelo
ModeloPreditor = ESN_MPC;

%% =============================================================================
% Define configurações do MPC
PassoMPC =3;                              % Proporção de amostras para atuação do Controlador (3 passos = 30s do processo)

%% ======================
% Parâmetros do Controlador (ainda por definir a melhor sintonia)
Hp = 6;                                % Horizonte de predição
Hc = Hp;                             % Horizonte de controle
Qy=  diag([1  10]);              % Qy - Peso das saidas controladas por setpoint = PChegada e Vazao)
Qu = diag([10  1]);              % Qu - Peso das ações de controle nas entradas (Alvos Desejados em  Freq. e PMonAlvo)
Qx= 0*diag(ones(1,11));    % Peso para os erros de estimação das  variáveis do processo
R=    0*diag([1  1]);             % R - Peso na variação das ações de controle - DeltaU em Freq. e PMonAlvo 

%% =============================================================================
% Para favorecer a inicialização e o tempo na busca da solução pelo Solver, vamos estabelecer limites minimos a máximos fixos
% para serem tratados em lbx/ubx.  As demais restrições serão tratadas em g

%                     PSuc   PChegada    PDiff    PDescarga   Tmotor      ITorque    ITotal     TSuc     Vibração   TChegada    Vazao
LimitesMin=   [  0             10               0               0                 0               0              0            0              0                 0                 0    ];    
LimitesMax=  [ 250          65            250            250             200          200         250         200           4               200           1000    ];

% Limites mín/max para as medições do processo (extraímos das tabelas Petrobras)
% LimitesMin= 0.9*[  55          10               50           120              20            40           40           20          0                 40            100    ];    
% LimitesMax=1.1*[ 110          65            150          215              141          140        183         141         3                150           800    ];


%% =============================================================================
% Considerando a diferença entre as grandezas, ajustamos os pesos para ter ua relação mais equilibrada
% na definição das matrizes de ponderação

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




