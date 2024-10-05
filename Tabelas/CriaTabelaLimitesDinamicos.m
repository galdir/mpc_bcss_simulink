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
  % OrdemCasadiBlock=[Psuc(1); Pcheg(1); Pdif(1); Pdesc(1); Tmotor(1); Ctorque(1); CTotal(1); Tsuc(1); Vib(1); Tche(1); Vazao(1)];
    Ordem= [ 6  10  8  7   1  5   4  2  3  12  13];
    TMax=[ round(Freq,1)  T(1,Ordem)];  
    TMin=[ round(Freq,1)  T(2,Ordem)];
    % OBS: este ROUND parece não fazer sentido, mas ao criar a tabela sem
    % ele, as vezes o Freq guardava na XLSX uma proximação na enésima casa decimal e gerava problemas na busca !!!
    TabelaLimites=[TabelaLimites; TMax; TMin];
end

% Criando a Tabela, mantendo os nomes da tabela original da petrobras
NomeVariaveis={'Frequencia','IntakePressure','ProductionSurfacePressure','DifferentialPressure',...
                            'DischargePressure','TMotor', 'TorqueCurrent', 'TotalCurrent','Tsuc', 'Vibracao',...
                            'ProductionSurfaceTemperature', 'VazaoOleo'};
varTypes={'double','double','double','double','double','double','double','double','double','double','double','double'};
N_lin=2*width(40:0.1:60);             % Numero de linhas
N_col=width(NomeVariaveis);     % Numero de colunas
sz=[N_lin,N_col];              % Dimensão da tabela
T= table('Size',sz,'VariableTypes',varTypes,'VariableNames',NomeVariaveis);
T{:,:}=TabelaLimites;
   
writetable(T,'TabelaLimitesDinamicos.xlsx');
   
