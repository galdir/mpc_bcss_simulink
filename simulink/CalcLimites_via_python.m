function Limites=CalcLimites_via_python(Freq,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa)
%
% Esta função retorna uma tabela com os valores limites para as variáveis do processo.
% Os limites podem ser extraídos das tabelas de proteção dinâmica, podem ter valores fixos (pré-defindos), 
% como também podem vir dos mapas de operação. 
%

%clear classes; %limpa imports anteriores do python para garantir que o import de modulos sejam das funcoes atualizadas
protecao_dinamica = py.importlib.import_module('codigos_python.protecao_dinamica');

protecoes_mapas = py.importlib.import_module('codigos_python.protecoes_mapas');
py.importlib.reload(py.importlib.import_module('codigos_python.protecoes_mapas'));

py.importlib.import_module('builtins');
pandas = py.importlib.import_module('pandas');

if nargin<1      % Não definiu argumento (só na fase de depuração)
    Freq=40;
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
% ProtecaoFixa =readtable('FixedProtections.xlsx');

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

%T = ProtecaoDinamica(Freq,TabRestricoesDinamicas,FxPercent); % define as restrições HARD dos estados em função da frequencia = obj.uk(1)'
TabRestricoesDinamicas_df = pandas.DataFrame(table_para_pydict(TabRestricoesDinamicas));
FxPercent__df = pandas.DataFrame(table_para_pydict(TabRestricoesDinamicas));
T_df = protecao_dinamica.calcula_protecao_dinamica(Freq,TabRestricoesDinamicas_df,FxPercent__df); % define as restrições HARD dos estados em função da frequencia = obj.uk(1)'
T = pandas_para_MatlabTable(T_df);
T{2,:}= [  0  T{1,2:10}.*(1-FxPercent{2,2:10})];               % Considera todos os percentuais associados ao alarme L
T{3,:}= [  0  T{1,2:10}.*(1+FxPercent{3,2:10})];              % Considera todos os percentuais associados ao alarme H

ProtecaoDin=table;                                                      % Criar tabela vazia e preencher com partes de outra tabela existente
ProtecaoDin=vertcat(ProtecaoDin,T(3,2:end));        % Extrai limites H das variáveis da tabela (sem a primeira coluna = Freq)
ProtecaoDin=vertcat(ProtecaoDin,T(2,2:end));        % Extrai limites L das variáveis da tabela (sem a primeira coluna = Freq)

%======================================
%  Gera tabela unificadas com todos os limites calculados
Limites=horzcat(ProtecaoFixa,ProtecaoDin);    % Unifica as tabelas com máximos e minimos de todas as variáveis

%============================
% PROTECAO 3 - Mapa de operação
gridP = 1;                          % Grid de pressão para maior precisão na interpolação dos limites dos mapas de operação
%[QMin,QMax,PSucMin,PSucMax,PCheMin,PCheMax]=ProtecoesMapas_via_python(TabSimulador,BTP,Freq,gridP);  % define as restrições suaves (estratégia por faixa do MPC) em função da frequencia
TabSimulador_df = pandas.DataFrame({table_para_pydict(TabSimulador)});
BTP_df = pandas.DataFrame({table_para_pydict(BTP)});
[QMin,QMax,PSucMin,PSucMax,PCheMin,PCheMax]=protecoes_mapas.calcula_protecoes_mapas(TabSimulador_df, BTP_df, Freq, gridP);  % define as restrições suaves (estratégia por faixa do MPC) em função da frequencia

% Necessário lembrar que no MAPA as contas de pressão são feitas em kgf/cm2
% ou seja, precisam ser convertidas para bar (1bar = 1.019716 kgf/cm2)
PSucMin=PSucMin/1.019716;
PSucMax=PSucMax/1.019716;
PCheMin=PCheMin/1.019716;
PCheMax=PCheMax/1.019716;

%=====================================================
% Substitui considerando os limites do mapa
% Limites da PChegada
Limites.ProductionSurfacePressure(1)=min(Limites.ProductionSurfacePressure(1),PCheMax);  % O limite máximo é o menor deles
Limites.ProductionSurfacePressure(2)=max(Limites.ProductionSurfacePressure(2),PCheMin);    % O limite mínimo é o maior deles
% Limites da PSuc
Limites.IntakePressure(1)=min(Limites.IntakePressure(1),PSucMax);  % O limite máximo é o menor deles
Limites.IntakePressure(2)=max(Limites.IntakePressure(2),PSucMin);   % O limite mínimo é o maior deles

end


