clc
clear all
close all

% Define sementes de números aleatórios para garantir repetibilidade
rand('seed',1) 
randn('seed',1)

% =============================================================================
% Necessário ajustar o Path do Matlab para apontar onde estão as tabelas e dados Petrobras e arquivos de uso geral
% Na pasta Uteis, há uma rotina AjustaPath.m

%% =============================================================================
% Carrega as tabelas e matrizes de referências da Petrobras para as proteções
CarregaTabelas; 

%% =============================================================================
% Carregar funções simbólicas que serão consumidas pelos códigos Matlab e CaSAdi
CarregaFuncoesSym;    
% f_buscaLimites_sym = Function('BuscaLimites', {Freq_sym}, {buscaLimitesMatriz_casadi_sym});
% f_Interpola_casadi_vazao_sym = Function('f_vazao', {Freq_sym, Press_sym}, {Interpola_casadi_vazao_sym});

%% =============================================================================
% Definição da base de tempo para simulação. 

% OBS: Simulação funciona com Ts diferente de 10s, mas isso faz sair da escala de tempo real e também faz errar o cálculo
% do volume produzido, já que a vazão estimada é em m3/dia. Assim, é importante manter Ts = 10
Ts =10;                                            % Passo de simulação (1 passo = 10s do processo) 

%% =============================================================================
% Limite para proporção na análise de tempo entre o tempo do Solver e a estimativa da ESN
TempoESN=0.0156;                % Tempo medido para a predição pela ESN na máquina Petrobras

% No ambiente simulado medimos a proporção entre o tempo do Solver e o tempo para a ESN que representa o modelo
% Assim , tendo esta proporção como referência e sabendo o TempoESN na máquina Petrobras, 
% sabemos um limite desta proporção para não violar Ts=10s

LimiteProporcao=Ts/TempoESN;    

%% =============================================================================
% Sabendo que os limites são calculados baseados na frequência e os valores são estabelecidos em função dos valores de 
% alarme L e H. Há variáveis, porém, cujos alarmes H ou L, por sí só, podem gerar TRIP da planta.
% Neste caso, definimos uma margem percentual como sendo uma região em que
% o controlador deve considerar como limite, antes de chegar no limite propriamente dito.
% O valor da margem é dado em % e pode ser ZERO.
% OBS: aplicamos estas margens percentuais apenas nas variáveis medidas do processo (estados X), 

MargemPercentual=1;     

%% =============================================================================
% Define condição inicial para a simulação de acordo com um instante com valores reais das variáveis do processo 
% 
% Observar que a condilçao inicial UIni (Freq e PMonAlvo) vem da condição do processo no instante selecionado
% e possivelmente serão diferentes dos alvos estabelecidos pelo usuário. 
% Isso não é crítico, mas influencia na inicialização da plotagem dos pontos alvos no mapa

%% Inicio de rampas de aceleração.
% DataHoraIni='2024-06-18 00:00:00';    % 4h, 60,32m3; Alvo=55Hz/35bar; PSuc=97.7    PChegada=33.08      Freq = 39,9Hz    4hPara rampa de aceleração
% DataHoraIni='2024-07-15 13:50:00';    % 4h, 60,9m3; Alvo=55Hz/35bar;  PSuc=96.7    PChegada=32.25      Freq = 40,3Hz    4h Para rampa de aceleração
% DataHoraIni='2024-07-17 00:00:00';    % 2h, 28,4m3; Alvo=55Hz/32bar; PSuc=97.4    PChegada=35.3      Freq = 40Hz    2h  Para rampa de aceleração

% Inicializações mais próximas da operação real
% Condição inicial  das variáveis do processo e das entradas      PSuc [bar],    PChegada [bar]        Freq [Hz]
DataHoraIni='2024-07-12 10:00:00';                                        % PSuc=77.4    PChegada=31.4      Freq = 54.9Hz  Ponto de operação usual
% DataHoraIni='2024-07-12 15:45:00';                                        % PSuc=78.9    PChegada=34.2      Freq = 53.9Hz   Ponto intermediário
% DataHoraIni='2024-12-01 00:00:00';                                            % PSuc=75.9    PChegada=31.89      Freq = 55.9Hz   Ponto intermediário

% Inicialização em uma rampa mas PSUC MUITO LONGE !!!
% DataHoraIni='2024-06-17 15:12:00';    % PSuc=149.2    PChegada=13.76      Freq = 40Hz     Para rampa de aceleração

[XIni,UIni]=SelCondicaoInicial(DataHoraIni,MatrizSimulador);         

%% Usa uma matriz h para seleção dos estados que vão compor a saida. Representa a função y=h(x) 
matriz_h=zeros(2,height(XIni)); % Tamanho da matriz que vai indicar as variáveis controladas por setpoint    
matriz_h(1,2)=1;            % PChegada - Coluna na linha 1 que indica a primeira variável controlada
matriz_h(2,11)=1;          % Vazao - Coluna na linha 2  que indica a segunda variável controlada 

% Só para lembrar do nome das variáveis e a ordem (coluna) delas nos estados X
% 1 = PSuc
% 2 = P.Chegada
% 3 = PDiff
% 4 = PDescarga
% 5 = TMotor
% 6 = ITorque
% 7 = ITotal
% 8 = TSuc
% 9 = Vibracao
% 10 = TChegada
% 11 = Vazao

%% =============================================================================
% Restrições máximas e mínimas para as variáveis manipuladas (entradas do processo)
FreqMaxMin=[60 ,  40];                                                  % Limites máx/min para ser dado pelo controlador como entrada de Freq no processo                           
PMonAlvoMaxMin=[50 , 20];                                          % Limites máx/min para ser dado pelo controlador como entrada de PMon no processo
umax  = [FreqMaxMin(1) ,  PMonAlvoMaxMin(1)];       % Vetor com valor máximo das manipuladas (Freq e PMonAlvo)
umin  =  [FreqMaxMin(2), PMonAlvoMaxMin(2)] ;        % Vetor com valor mínimo das manipuladas  (Freq e PMonAlvo)
 
% Delta U - variação máxima permitida nas ações de controle (Freq e PMonAlvo)
dumax = [0.25 , 2];                                                       %Variação máxima nas manipuladas [ Hz    bar ]

%% =============================================================================
% Carrega tabela com plano de experimentos programados para mudanças automáticas de Freq. e PMonAlvo para simulação
% Observar que nos demais casos há proteção para que não se coloque alvo ENG em regiões proibidas. No caso do PLANO, isso não é testado e
% o plano será executado tal qual definido em tabela, mesmo com alvo em região proibida

Plano=readtable('PlanoOperacao.xlsx');           % Plano para avaliar caminho da máxima vazão

% Define se vai usar plano (tabela excel) para alterar alvos da engenharia ao longo da simulação
UsaPlano=0;
if UsaPlano    % Sequencia para usar plano definido em planilha
    %     Inicializa o alvo da ENG como sendo o primeiro registro do plano proposto
    FreqAlvoIni=Plano.Frequencia(1);                    % Resgata da tabela o ponto de inicial desejado pela ENG para a Frequencia [Hz]
    PMonAlvoIni=Plano.PMonAlvo(1);                    % Resgata da tabela o ponto de inicial desejado pela ENG para a PMonAlvo [bar]
    StopTime=Plano.Tempo(end);                          % O tempo de simulação segue o plano definido na tabela 
else              % Se não usa plano da tabela, precisa de alvo (Freq e PMonAlvo)  definidos automaticamente ou manualmente
    StopTime=4*3600;          % Define manualmente um tempo para a simulação, lembrando que 3600s=1h
    AlvoAutomatico=1;          % 1/0 para definir se vai usar alvo automático ou alvo manualmente fornecido pela engenharia
    if AlvoAutomatico             % 
        FreqAlvoIni=57;           % Não aguarda definição da engenharia e aponta para a frequência específica
        Limites= full(f_buscaLimites_sym(FreqAlvoIni)); 
        LimiteVazao=Limites(1,11)*(1-MargemPercentual/100);     % Limite de upthrust considerada a Margem Percentual Configurada
        PMonAlvoIni=max([ Limites(2,2), PMonAlvoMaxMin(2)]);     % Mais conservador entre limite minimo (linha 2) da PChegada (coluna 2) ou a PMonAlvoMin definida
        PMonAlvoIni=PMonAlvoIni*(1+MargemPercentual/100);      % Aponta para uma região alcançavel e não no limite do Upthrust
        VazaoAlvo=Interpola(FreqAlvoIni,PMonAlvoIni*1.019716,MatrizSimulador,3);   
        while VazaoAlvo>LimiteVazao                                                % Enquanto a PMonAlvo apontar para uma violação dos limites da Vazao
            PMonAlvoIni=PMonAlvoIni+0.01;                                        % Incrementa PMonAlvoIni para que a Vazão chegue no limite da margem % configurada
            VazaoAlvo=Interpola(FreqAlvoIni,PMonAlvoIni*1.019716,MatrizSimulador,3);   % Calcula o que será a Vazão Alvo da Engenharia
        end
    else    % Os alvos serão dados manualmente (usamos em teste)
        %    Inicializa alvo da ENG manualmente (2 casos básicos)
        AlvosRefFreq=  [ UIni(1)  56   57   57    58  ];
        AlvosRefPMon=[ UIni(2)  25   27   30    30  ]; 
        Caso=4;
         
        % Seleciona FreqAlvo e PMonAlvo
        FreqAlvoIni=AlvosRefFreq(Caso);          % Tem de estar na faixa de 40 a 60Hz !! Criar proteção na implementação Python
        % Avalia valores dados manualmente calcula limites da PChegada em função do mapa
        % Com base nestas contas, não deixa setar alvos ENG fora de regiões úteis do mapa 
        PMonAlvoIni=AlvosRefPMon(Caso);    % Cuidado !!! Aqui a engenharia pode setar um valor em área "proibida" 
    end
end

%% =============================================================================
% Carrega uma rede (ESN qualquer) para ser usada como modelo do processo
% Rede_Processo = load('weightsESNx_TR300_TVaz0.8_RaioE0.1.mat');
% Rede_Processo = load('weightsESNx_TR400_TVaz0.9_RaioE0.4.mat');
% Rede_Processo = load('weightsESNx_TR900_TVaz0.9_RaioE0.4.mat');
% Rede_Processo = load('6-weightsESNx_TR200_TVaz0.50_RaioE0.99_mape_msa_dois_2.71.mat');
Rede_Processo=load('2-weightsESNx_TR200_TVaz0.50_RaioE0.50-treino_070809102024-mape_msa_070809102024_1.77.mat');

% Número de casas decimais para corresponder a resolução dos instrumentos
NumCasasDecimais=5;

% Inserir ruido na saida do processo para simular mundo real e avaliar robustez do controlador
% SNR = 20;   % Relação sinal ruido para um ruido gaussiano aditivo à ser aplicado nas variáveis do modelo
SNR = 100;   % Relação sinal ruido para um ruido gaussiano aditivo à ser aplicado nas variáveis do modelo
% SNR = 40;   % Relação sinal ruido para um ruido gaussiano aditivo à ser aplicado nas variáveis do modelo
% Uma relação sinal-ruído (SNR) de 1 dB significa que a potência do sinal é igual a potência do ruído. 
% Uma relação sinal-ruído (SNR) de 10 dB significa que o sinal é 10 vezes mais potente que o ruído. 
% Uma relação sinal-ruído (SNR) de 20 dB significa que o sinal é 100 vezes mais potente que o ruído. 
% Uma relação sinal-ruído (SNR) de 30 dB significa que o sinal é 1000 vezes mais potente que o ruído. 

%% =============================================================================
disp('Configurações para a simulação foram carregadas para a área de trabalho')
disp('Funções simbólicas carregadas para a área de trabalho')



