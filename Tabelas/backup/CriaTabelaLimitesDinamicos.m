clear all
clc

TabRestricoesDinamicas=table2array(readtable('DP.xlsx'));              % Tabela de valores de referência para as variáveis

FxPercent=readtable('DP-Faixas.xlsx');            % Tabela das faixas percentuais para disparar os alarmes LL, L, H e HH
FxPercent=table2array(FxPercent(:,2:end));     % Extrai a primeira coluna com os simbolos dos alarmes
FxPercent=[ [0;0;0;0]    FxPercent ];                    % Preenche a primeira coluna com zeros para manter o mesmo indice da tabela DP

BTP=table2array(readtable('DoBTP.xlsx'));                                           % Tabela de dados do teste de produção

ProtecaoFixa =readtable('FixedProtections.xlsx');            % Carrega tabelas de proteção fixa da Petrobras
ProtecaoFixa=table2array(ProtecaoFixa(:,2:end));           % Extrai a primeira coluna da tabela com simbolos de Max/Min

TabSimulador=table2array(LeConverteNomes('DoSimulador.xlsx'));  % Tabela do simulador (Análise de Sensibilidade)

TabelaLimites=[];
Freq=40;

for Freq=40:0.1:60 
    T=CalcLimites(Freq,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa);
    
    % 3 Proteções fixas da tabela original Petrobras
    % Coluna1 = TMotor
    % Coluna2 = TSuc
    % Coluna3 = Vibracao

    % 9 Protecoes da tabela de restrições dinâmicas original Petrobras
    % Coluna4 = TotalCurrent
    % Coluna5 = TorqueCurrent
    % Coluna6 = IntakePressure   (PSuc)
    % Coluna7 = DischargePressure (PDescarga)
    % Coluna8 = DifferentialPressure  (PDiff)
    % Coluna9 = XTreePressure (Pressão medida pelo TPT)  -  NÃO USAMOS
    % Coluna10 = ProductionSurfacePressure (PChegada)
    % Coluna11 = DownholePressure  (Pressão medida pelo PDG)  -  NÃO USAMOS
    % Coluna12 = ProductionSurfaceTemperature ( Temperatura de Chegada)

    % 1 inserida sendo a estimativa da vazão
    % Coluna 13 = Vazao de Oleo inserida com a vazão estimada
    
    % Seleciona apenas as variáveis disníbeis e que serão usadas (10 processo + 3 fixas + 1 Vazao) 
  % Ordem Variáveis =[Psuc(1); Pcheg(1); Pdif(1); Pdesc(1); Tmotor(1); Ctorque(1); CTotal(1); Tsuc(1); Vib(1); Tche(1); Vazao(1)];
    Ordem= [ 6  10  8  7   1  5   4  2  3  12  13];
    TMax=[ round(Freq,1)  T(1,Ordem)];  
    TMin=[ round(Freq,1)  T(2,Ordem)];
    % OBS: este ROUND parece não fazer sentido, mas ao criar a tabela sem
    % ele, as vezes o Freq guardava na XLSX uma proximação na enésima casa decimal e gerava problemas na busca !!!
    TabelaLimites=[TabelaLimites; TMax; TMin];
end

% Criando a Tabela, mantendo os nomes da tabela original da petrobras
NomeVariaveis={'Frequencia','PSuc','PChegada','PDiff',...
                            'PDescarga','TMotor', 'ITorque', 'ITotal','TSuc', 'Vibracao',...
                            'TChegada', 'VazaoOleo'};
varTypes={'double','double','double','double','double','double','double','double','double','double','double','double'};
N_lin=2*width(40:0.1:60);             % Numero de linhas
N_col=width(NomeVariaveis);     % Numero de colunas
sz=[N_lin,N_col];              % Dimensão da tabela
T= table('Size',sz,'VariableTypes',varTypes,'VariableNames',NomeVariaveis);
T{:,:}=TabelaLimites;
   
writetable(T,'TabelaLimitesDinamicos.xlsx');
%=============================================================
function Limites=CalcLimites(Freq,MatrizSimuladorContas,BTP,MatrizRestricoesDinamicas,FxPercent,ProtecaoFixa);
    % Esta função retorna uma matriz os valores limites para as variáveis do processo.
    % Os limites podem ser extraídos das tabelas de proteção dinâmica, podem ter valores fixos (pré-defindos), 
    % como também podem vir dos mapas de operação. 

    %=================================
    % PROTECAO FIXA - Proteções pré-definidas
    % De uma tabela definida pela Petrobras, usamos os limites fixos para as seguintes variáveis:
    %  1 - Limites H e L da Temperatura do Motor
    %  2 - Limites H e L da Temperatura de Sucção
    %  3 - Limites H e L da Vibração

    %===============================
    % PROTECAO 2 - Proteções Dinâmicas
    % Da tabela de proteção dinâmica da Petrobras, usamos com limites das variáveis:
    %  1 - Limites de alarme H e L da Corrente Total da BCSS
    %  2 - Limites de alarme H e L da Corrente de Torque da BCSS
    %  3 - Limites de alarme H e L  da Pressão de Sucção de BCSS
    %  4 - Limites de alarme H e L  da Pressão de Descarga da BCSS
    %  5 - Limites de alarme H e L  da Pressão Diferencial da BCSS
    %  6 - Limites de alarme H e L da Pressão de Chegada
    %  7 - Limites de alarme H e L da Temperatura de Chegada

    ProtecaoDin = ProtecaoDinamica(Freq,MatrizRestricoesDinamicas,FxPercent); % Define as restrições HARD dos estados em função da frequencia
    ProtecaoDin=ProtecaoDin(:,2:end);     % Extrai apenas os limites Max/Min (exclui a primeira coluna = Frequencia])
        % Coluna1 = TotalCurrent
        % Coluna2 = TorqueCurrent
        % Coluna3 = IntakePressure   (PSuc)
        % Coluna4 = DischargePressure (PDescarga)
        % Coluna5 = DifferentialPressure  (PDiff)
        % Coluna6 = XTreePressure (Pressão medida pelo TPT)
        % Coluna7 = ProductionSurfacePressure (PChegada)
        % Coluna8 = DownholePressure  (Pressão medida pelo PDG)
        % Coluna9 = ProductionSurfaceTemperature ( Temperatura de Chegada)
    %============================
    % PROTECAO 3 - Mapa de operação
    [QMin,QMax,PSucMin,PSucMax,PCheMin,PCheMax]=ProtecoesMapas(MatrizSimuladorContas,BTP,Freq);  % define as restrições suaves (estratégia por faixa do MPC) em função da frequencia

    % Necessário lembrar que no MAPA as contas de pressão foram feitas em kgf/cm2
    % ou seja, precisam ser convertidas para bar (1bar = 1.019716 kgf/cm2)
    PSucMin=PSucMin/1.019716;
    PSucMax=PSucMax/1.019716;
    PCheMin=PCheMin/1.019716;
    PCheMax=PCheMax/1.019716;

    %=====================================================
    % Substitui considerando os limites do mapa
    % Limites da PChegada (Coluna 7 na matriz de Proteções Dinâmicas original da Petrobras)
    ProtecaoDin(1,7)=min(ProtecaoDin(1,7),PCheMax);    % O limite máximo é o menor deles
    ProtecaoDin(2,7)=max(ProtecaoDin(2,7),PCheMin);    % O limite mínimo é o maior deles
    % Limites da PSuc  (Coluna 3 na matriz de Proteções Dinâmicas original da Petrobras)
    ProtecaoDin(1,3)=min(ProtecaoDin(1,3),PSucMax);     % O limite máximo é o menor deles
    ProtecaoDin(2,3)=max(ProtecaoDin(2,3),PSucMin);     % O limite mínimo é o maior deles


    %======================================
    %  Gera tabela unificada contendo todos os limites (fixos e dinâmicos) que foram calculados
    Limites=horzcat(ProtecaoFixa,ProtecaoDin);    % Unifica as tabelas com máximos e minimos de todas as variáveis

    %=====================================================
    % Insere limites de Vazão Max/Min
    Limites=horzcat(Limites,[QMax;QMin]);   % Cria coluna Vazão com os limites calculados

end