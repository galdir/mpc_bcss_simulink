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
Qy=  diag([1  1]);        % Qy - Peso das Controladas (PSuc e PChegada) prioridade da variável controlada voltar para respectiva faixa (calculada pelas proteções dinâmicas)
Qu = diag([1  1]);        % Qu - Peso dos Alvos Desejados (Freq. e PMonAlvo). Ponderação priorização dos alvos (ponto do mapa)
R=    [1  1];                  % R - Peso na variação da ação de controle - Delta U em (Freq. e PMonAlvo) 
R =  diag(repmat(R,1,Hc));   % Para diminuir/aumentar o custo de controle das variáveis manipuladas (Freq. e PmonAlvo) em todo o Hc


% Inicializa estados para atuação do MPC
MatrizYIni= repmat(YIni,1,Hp+1);                            % Condição inicial das variáveis (preenche horizonte até Hp+1 com valores iniciais)
DeltaUIni= repmat(dumax,Hc,1);                             % Condição incial dos deltaU para todo o horizonte de controle
InicializaMPC=[MatrizYIni(:);DeltaUIni;YIni(1:2)];   % Monta vetor de estados iniciais para operação do MPC
% YIni são as condições iniciais das 10 variáveis do processo
% DeltaUIni são as varições para cada variável manipulada (poderia inicializar com zeros, mas é indiferente pois o Delta U será calculado)
% YIni(1:2), São a PSuc e a PChegada, ou seja, é a condição inicial das variáveis controladas

%% =============================================================================
disp('Configurações e parâmetros do controlador foram carregados para a área de trabalho')
%% =============================================================================
% Fim da rotina principal
