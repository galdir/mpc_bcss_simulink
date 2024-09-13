function [Tt,T,Inicio,Fim]=DadosBase (NomeArq, Inicio,Fim);
% Função para carregar os dados de uma tabela parquet e já organizar num
% formato para ser aproveitado pelos modelos no ambiente do Simulink
% 
% Retorna a tabela parquet e respectivo array que será consumido como
% entrada de dados nos ambientes de simulação
%
% Além de carregar os dados, inclui informação de Falha no alinhamento das
% válvulas, assim como, falha no período de amostragem (diferente de 11 ou
% 12s).

% =============================================================================
% Prepara leitura de dados do arquivo parquet
Tt=parquetread(NomeArq);
Tt=fillmissing(Tt,'previous');

% Se não passou os parâmetros da janela de tempo, deixa o arquivo completo
% Caso contrário, carrega apenas a janela de tempo especificada
if nargin>1
    Tt = Tt(Tt.data_hora >= Inicio & Tt.data_hora <= Fim, :);    % Extrai os valores na faixa de tempo selecionada
end

Inicio=Tt.data_hora(1);    % Retorna data_hora de inicio efetivo
Fim=Tt.data_hora(end);   % Retorna data_hora de final efetivo


% Avalia condições para a operação. Indica o caso de 
%      - Alinhamento das válvulas não associar produção
%      - Amostragem estar oscilando (aceito apenas Ts = 11 ou 12s )
% Cria colunas extras para guardar a condição de falha no alinhamento ou
% falha no período de amostragem da janela selecionada (o MPA habilitado já
% tem coluna na tabela original)

% Avalia os flags que indicam a operação
F=flags_operacao(Tt);                    
Tt.('Falha_Alinhamento')=F(:,1);      % Extrai flags do alinhamento das válvulas
Tt.('Falha_Amostragem')=F(:,3);     % Extrai flags de problemas na amostragem

% Desnecessário pois a informação de MPA habilitado já existe na tabela original 
%Tt.('Falha_Amostragem')=F(:,2);     % Extrai flags de MPA habilitado

% Extrai a informação de data_hora pois a tabela será convertida para Array
Tt = removevars(Tt,'data_hora');     

% Seleciona as variáveis que vamos mostrar na simulação
NomeVariavel={'choke_producao','frequencia_BCSS','pressao_montante_alvo',...
    'pressao_chegada','pressao_succao_BCSS','temperatura_chegada',...
    'pressao_descarga_BCSS','pressao_diferencial_BCSS', 'temperatura_succao_BCSS',...
    'temperatura_motor_BCSS','corrente_torque_BCSS','corrente_total_BCSS',...
    'vibracao_BCSS','habilita_mpa','Falha_Alinhamento','Falha_Amostragem'};

Tt=Tt(:,NomeVariavel);             % Extrai apenas as variáveis de interesse 

T=table2array(Tt);                   % Converte as variáveis de interesse para array

% Mas para funcionar no simulink, a primeira coluna precisa ser o tempo,
% Lembrando que no nosso caso, nossa unidade de tempo é contado em amostras

NumCol=width(T);                                         % Avalia o numero de colunas
NumReg=height(T);                                      % Guarda numero de registros
T(:,2:NumCol+1)=T(:,1:NumCol);                 % Desloca as colunas para a direita
T(:,1)=0:NumReg-1;                                      % Insere uma sequencia de 1 em 1 na primeira coluna (primeira coluna é considerada como tempo no Simulink)

% =============================================================================