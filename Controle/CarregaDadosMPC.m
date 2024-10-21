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
Hp = 3;                            % Horizonte de predição
Hc = 2;                            % Horizonte de controle
Qy=  diag([1  1]);             % Qy - Peso das saidas controladas por setpoint = PChegada e Vazao)
Qx= diag(ones(1,11));    % Peso para os erros de estimação das  variáveis do processo
Qu = diag([1  1]);             % Qu - Peso das ações de controle nas entradas (Alvos Desejados = Freq. e PMonAlvo)
R=    diag([1  1]);              % R - Peso na variação das ações de controle - Delta U em (Freq. e PMonAlvo) 

%% =============================================================================
% Para favorecer a inicialização e o tempo na busca da solução pelo Solver, vamos estabelecer limites minimos a máximos fixos
% para serem tratados em lbx/ubx.  As demais restrições serão tratadas em g

% Limites mín/max para as medições do processo (extraímos das tabelas Petrobras)
%                   PSuc  PChegada    PDiff    PDescarga   Tmotor    ITorque    ITotal     TSuc   Vibração   TChegada    Vazao
LimitesMin= [  55          10               50           120              20            40           40           20          0                 40            100    ];    
LimitesMax=[ 110          65            150          215              141          140        183         141         3                150           800    ];

%% =============================================================================
    
disp('Configurações e parâmetros do controlador MPC foram carregados para a área de trabalho')
%% =============================================================================
% Fim da rotina principal

% Só para saber das dimensões
% nu=length(Qu);
% ny=length(Qy);
% nx=length(Qx);
%  nx_ESN = length(ModeloPreditor.data.a0);
 
% DimensaoX = length(Qx)*(1+Hp) + length(Qu)*Hp);

%                                 [  Medições  AlvoEng;    Ysp     ErroX        ErroY    BuffDeltaFreq      Reservatório do ModeloPreditor]
% DimensaoP=sum([        nx             nu            ny            nx             ny            15                       nx_ESN                  ]);





