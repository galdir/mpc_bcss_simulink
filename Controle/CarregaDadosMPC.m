% Carrega para a área de trabalho dados e parâmetros gerais para atuação do controlador MPC

% Começa carregando dados de uso geral e comum a qualquer ambiente (MPC ou CBR)
CarregaDados;      % Função para carregar tabelas Petrobras e dados gerais

%% =============================================================================
% Escolha o modelo Preditor no MPC
TipoRede=1;        % 1=ESN ou  2=LSTM 

% Carrega rede ESN que será utilizada como preditor no MPC
% ESN_MPC = load('weightsESNx_JUB27n100.mat');
% ESN_MPC = load('weightsESNx_TR300_TVaz0.8_RaioE0.1.mat');    % Esta foi usada como processo
ESN_MPC = load('weightsESNx_TR400_TVaz0.9_RaioE0.4.mat');
ESN_MPC.data.tipo = TipoRede;     % Insere o tipo de rede na estrutura do modelo
ModeloPreditor = ESN_MPC;

% ==================
% Carrega o modelo LSTM
% pasta_modelos = '.\modelos_lstm2_casadi\';
% LSTM_MPC = carregaModelosCasadi(pasta_modelos);
% % Falta inserir o TIPO = 2
% ModeloPreditor = LSTM_MPC;

%% =============================================================================
% Define configurações do MPC
PassoMPC =3;                              % Proporção de amostras para atuação do Controlador (3 passos = 30s do processo)

%% ======================
% Parâmetros do Controlador (ainda por definir a melhor sintonia)
Hp =  10;                     % Horizonte de predição
Hc =  4 ;                      % Horizonte de controle
Qy=  diag([1  1]);        % Qy - Peso das saidas controladas por setpoint = PSuc e PChegada)
Qx= 1;                         % Peso para os erros de estimação das  variáveis do processo
Qu = diag([1  1]);        % Qu - Peso das ações de controle nas entradas (Alvos Desejados = Freq. e PMonAlvo)
R=    [1  1];                  % R - Peso na variação das ações de controle - Delta U em (Freq. e PMonAlvo) 
R =  diag(repmat(R,1,Hc));   % Para ponderar o custo de variação nas manipuladas (Freq. e PmonAlvo) em todo o Hc
% Ver como levar esta expansão de R em todo Hc para dentro da inicialização

%  % Usa uma matriz h para seleção dos estados que vão compor a saida. Representa a função y=h(x) 
% matriz_h=zeros(2,height(XIni)); % Tamanho da matriz que vai oferecer a saida do sistema    
% matriz_h(1,1)=1;            % Coluna na linha 1 que indica a primeira variável controlada
% matriz_h(2,2)=1;            % Coluna na linha 2  que indica a segunda variável controlada 
% 
% % Só para lembrar do nome das variáveis e a ordem (coluna) delas nos estados X
% % 1 = PSuc
% % 2 = P.Chegada
% % 3 = PDiff
% % 4 = PDescarga
% % 5 = TMotor
% % 6 = ITorque
% % 7 = ITotal
% % 8 = TSuc
% % 9 = Vibracao
% % 10 = TChegada
% % 11 = Vazao
% 

%% =============================================================================
disp('Configurações e parâmetros do controlador foram carregados para a área de trabalho')
%% =============================================================================
% Fim da rotina principal
