function Limites=CalcLimites(Freq,MatrizSimuladorContas,BTP,MatrizRestricoesDinamicas,FxPercent,ProtecaoFixa);
%
% Esta função retorna uma matriz os valores limites para as variáveis do processo.
% Os limites podem ser extraídos das tabelas de proteção dinâmica, podem ter valores fixos (pré-defindos), 
% como também podem vir dos mapas de operação. 
%

if nargin<1      % Não definiu argumento (só na fase de depuração)
    Freq=55;
end
if nargin<6       % Não passou as tabelas como parâmetros
    % Carrega as tabelas de referências para as proteções dinâmicas
    CarregaTabelas;       % Carrega as tabelas de referências para as proteções
end

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

% Necessário lembrar que no MAPA as contas de pressão são feitas em kgf/cm2
% ou seja, precisam ser convertidas para bar (1bar = 1.019716 kgf/cm2)
PSucMin=PSucMin/1.019716;
PSucMax=PSucMax/1.019716;
PCheMin=PCheMin/1.019716;
PCheMax=PCheMax/1.019716;

%=====================================================
% Substitui considerando os limites do mapa
% Limites da PChegada (Coluna 7 na matriz de Proteções Dinâmicas original da Petrobras)
ProtecaoDin(1,7)=min(ProtecaoDin(1,7),PCheMax);  % O limite máximo é o menor deles
ProtecaoDin(2,7)=max(ProtecaoDin(2,7),PCheMin);    % O limite mínimo é o maior deles
% Limites da PSuc  (Coluna 3 na matriz de Proteções Dinâmicas)
ProtecaoDin(1,3)=min(ProtecaoDin(1,3),PSucMax);  % O limite máximo é o menor deles
ProtecaoDin(2,3)=max(ProtecaoDin(2,3),PSucMin);   % O limite mínimo é o maior deles


%======================================
%  Gera tabela unificada contendo todos os limites (fixos e dinâmicos) que foram calculados
Limites=horzcat(ProtecaoFixa,ProtecaoDin);    % Unifica as tabelas com máximos e minimos de todas as variáveis

%=====================================================
% Insere limites de Vazão Max/Min
Limites=horzcat(Limites,[QMax;QMin]);   % Cria coluna Vazão com os limites calculados
