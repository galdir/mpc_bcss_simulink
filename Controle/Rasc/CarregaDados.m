clc
clear all
close all

% =============================================================================
% Necessário ajustar o Path do Matlab para apontar onde estão as tabelas e dados Petrobras e arquivos de uso geral
% Na pasta Uteis, há uma rotina AjustaPath.m

%% =============================================================================
% Carrega as tabelas e matrizes de referências da Petrobras para as proteções
CarregaTabelas; 

%% =============================================================================
% Carregar funções simbólicas que serão consumidas pelos códigos Matlab e CaSAdi
CarregaFuncoesSym;    

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
% Define condição inicial para a simulação de acordo com um instante com valores reais das variáveis do processo 
% 
% Observar que a condilçao inicial UIni (Freq e PMonAlvo) vem da condição
% do processo no instante selecionado e possivelmente serão diferentes dos alvos estabelecidos pelo usuário. 
% Isso não é crítico, mas influencia na inicialização da plotagem dos pontos alvos no mapa

% Condição inicial  das variáveis do processo e das entradas     PSuc [bar],    PChegada [bar]        Freq [Hz]
%  [XIni,UIni]=SelCondicaoInicial('2024-07-12 10:00:00',MatrizSimulador);     % PSuc=77.4    PChegada=31.4      Freq = 54.9Hz  Ponto de operação usual
% [XIni,UIni]=SelCondicaoInicial('2024-07-12 15:45:00',MatrizSimulador);       % PSuc=78.9    PChegada=34.2      Freq = 53.9Hz   Ponto intermediário
% [XIni,UIni]=SelCondicaoInicial('2024-06-17 15:12:00',MatrizSimulador);       % PSuc=149.2    PChegada=13.76      Freq = 40Hz     Para rampa de aceleração

% Inicio de rampas de aceleração para comparação
% [XIni,UIni]=SelCondicaoInicial('2024-06-18 00:00:00',MatrizSimulador);        % 4h, 60,32m3; Alvo=55Hz/35bar; PSuc=97.7    PChegada=33.08      Freq = 39,9Hz    4hPara rampa de aceleração
[XIni,UIni]=SelCondicaoInicial('2024-07-15 13:50:00',MatrizSimulador);        % 4h, 60,9m3; Alvo=55Hz/35bar;  PSuc=96.7    PChegada=32.25      Freq = 40,3Hz    4h Para rampa de aceleração
% [XIni,UIni]=SelCondicaoInicial('2024-07-17 00:00:00',MatrizSimulador);            % 2h, 28,4m3; Alvo=55Hz/32bar; PSuc=97.4    PChegada=35.3      Freq = 40Hz    2h  Para rampa de aceleração

 % Usa uma matriz h para seleção dos estados que vão compor a saida. Representa a função y=h(x) 
matriz_h=zeros(2,height(XIni)); % Tamanho da matriz que vai oferecer a saida do sistema    
matriz_h(1,1)=1;            % Coluna na linha 1 que indica a primeira variável controlada
matriz_h(2,2)=1;            % Coluna na linha 2  que indica a segunda variável controlada 

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

% AVALIAR SE VALE A PENA MUDAR A ORDEM DE TODA A CALC_LIMITES
% Talvez possamos usar a matriz_h para facilitar a extração dos limites (alarmes),
% Só para lembrar a ordem (coluna) que das variáveis que retornam da CalcLimites atual
% 1 = TMotor
% 2 = TSuc
% 3 = Vibracao
% 4 = ITotal
% 5 = ITorque
% 6 = PSuc
% 7 = PDescarga
% 8 = PDiff
% 9 = P.Chegada
% 10 = TChegada
% 11 = Vazao

%% =============================================================================
% Restrições máximas e mínimas para as variáveis manipuladas (entradas do processo)

FreqMaxMin=[60; 40];                                                  % Limites máx/min para ser dado pelo controlador como entrada de Freq no processo                           
% PMonAlvoMaxMin=[40; 25];                                     % Limites máx/min para ser dado pelo controlador como entrada de PMon no processo
PMonAlvoMaxMin=[50; 20];                                         % Limites máx/min para ser dado pelo controlador como entrada de PMon no processo
umax  = [FreqMaxMin(1); PMonAlvoMaxMin(1)];       % Vetor com valor máximo das manipuladas (Freq e PMonAlvo)
umin  =  [FreqMaxMin(2); PMonAlvoMaxMin(2)] ;      % Vetor com valor mínimo das manipuladas  (Freq e PMonAlvo)
 
% Delta U - variação máxima permitida nas variáveis manipuladas
dumax = [0.1; 1];                                                       %Variação máxima nas manipuladas [ Hz    bar ]
dumin = [0 ; 0];                                                          %Variação mínima nas manipuladas [ Hz    bar ]
%dumin = -dumax;

%% =============================================================================
% Carrega tabela com plano de experimentos programados para mudanças automáticas de Freq. e PMonAlvo para simulação
% Observar que nos demais casos há proteção para que não se coloque alvo ENG em regiões proibidas. No caso do PLANO, isso não é testado e
% o plano será executado tal qual definido em tabela, mesmo com alvo em região proibida

% Plano=readtable('PlanoTesteCampoDia12.07.2024.xlsx');    % Plano que reproduz testes de campo no dia 12/07/2024, neste caso, usamos a inicialização do dia 12/07 às 10h
% Plano=readtable('PlanoVerIsovazao.xlsx');     % Plano com partida "puxando" para menores valores de PChegada induzindo caminho de maior produção
% Plano=readtable('PlanoAceleracao.xlsx');     % Plano com partida "puxando" para menores valores de PChegada induzindo caminho de maior produção
 Plano=readtable('PlanoAceleracaoErro.xlsx');     % Plano com partida "puxando" para menores valores de PChegada induzindo caminho de maior produção

% Define se vai usar plano (tabela excel) para alterar alvos da engenharia ao longo da simulação
UsaPlano=0;
if UsaPlano    % Sequencia para usar plano definido em planilha
    %     Inicializa o alvo da ENG como sendo o primeiro registro do plano proposto
    FreqAlvoIni=Plano.Frequencia(1);                    % Resgata da tabela o ponto de inicial desejado pela ENG para a Frequencia [Hz]
    PMonAlvoIni=Plano.PMonAlvo(1);                    % Resgata da tabela o ponto de inicial desejado pela ENG para a PMonAlvo [bar]
    StopTime=Plano.Tempo(end);                          % O tempo de simulação segue o plano definido na tabela 
%     StopTime=2000; 
else              % Se não usa plano da tabela, precisa de alvo (Freq e PMonAlvo)  definidos automaticamente ou manualmente
    StopTime=4*3600;          % Define manualmente um tempo para a simulação, lembrando que 3600s=1h
    AlvoAutomatico=1;          % 1/0 para definir se vai usar alvo automático ou alvo manualmente fornecido pela engenharia
    if AlvoAutomatico             % 
        FreqAlvoIni=60;           % Não aguarda definição da engenharia e aponta para a frequência máxima possível
        Limites= full(f_buscaLimites_sym(FreqAlvoIni)); 
        PMonAlvoIni=Limites(2,2);     % Extrai ponto limite minimo (linha 2) da PChegada (coluna 2)
    else                                  % Os alvos serão dados manualmente pela engenharia
        %    Inicializa alvo da ENG manualmente
        FreqAlvoIni=55;          % Tem de estar na faixa de 40 a 60Hz !! Criar proteção na implementação Python
        % Avalia valores dados manualmente calcula limites da PChegada em função do mapa
        % Com base nestas contas, não deixa setar alvos ENG fora de regiões úteis do mapa 
        PMonAlvoIni=32;    % Aqui a engenharia pode setar um valor em área "proibida". Vamos proteger !!
        Limites= full(f_buscaLimites_sym(FreqAlvoIni)); 
        FaixaPChegada=Limites(:,2);      % Extrai faixa [ Max  Min] da PChegada (coliuna 2) em bar
        PMonAlvoIni=LimitaFaixa(PMonAlvoIni,FaixaPChegada);  % Limita a variável nos limites definidos. Se for dentro da faixa, não muda em nada o que foi definido pela ENG
    end
end

%% =============================================================================
% Carrega uma rede (ESN qualquer) para ser usada como modelo do processo
% Rede_Processo = load('weightsESNx_JUB27n100.mat');
Rede_Processo = load('weightsESNx_TR300_TVaz0.8_RaioE0.1.mat');
% Rede_Processo = load('weightsESNx_TR400_TVaz0.9_RaioE0.4.mat');
% Rede_Processo = load('weightsESNx_TR900_TVaz0.9_RaioE0.4.mat');

% Número de casas decimais para corresponder a resolução dos instrumentos
NumCasasDecimais=1;

% Inserir ruido na saida do processo para simular mundo real e avaliar robustez do controlador
% SNR = 20;   % Relação sinal ruido para um ruido gaussiano aditivo à ser aplicado nas variáveis do modelo
SNR = 40;   % Relação sinal ruido para um ruido gaussiano aditivo à ser aplicado nas variáveis do modelo
% Uma relação sinal-ruído (SNR) de 1 dB significa que a potência do sinal é igual a potência do ruído. 
% Uma relação sinal-ruído (SNR) de 10 dB significa que o sinal é 10 vezes mais potente que o ruído. 
% Uma relação sinal-ruído (SNR) de 20 dB significa que o sinal é 100 vezes mais potente que o ruído. 
% Uma relação sinal-ruído (SNR) de 30 dB significa que o sinal é 1000 vezes mais potente que o ruído. 

%% =============================================================================
% Sabendo que os limites são calculados baseados na frequência e os valores são estabelecidos em função dos valores de 
% alarme L e H. Há variáveis, porém, cujos alarmes H ou L, por sí só, podem gerar TRIP da planta.
% Neste caso, definimos uma margem percentual como sendo uma região em que
% o controlador deve considerar como limite, antes de chegar no limite propriamente dito.
% O valor da margem é dado em % e pode ser ZERO
MargemPercentual=0.5;

%% =============================================================================
disp('Configurações gerais carregadas para a área de trabalho')
disp('Funções simbólicas carregadas para a área de trabalho')


