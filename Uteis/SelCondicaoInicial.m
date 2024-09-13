function [YIni,UIni]=SelCondicaoInicial(DIni);
% Retorna condição inicial na data_hora pré-selecionada para inicio da simulação do controle. 
% Inclui as condilçoes iniciais das variáveis do processo (YIni) e das variáveis manipuladas (UIni)

%=============================
if nargin<1      % Caso o usuário não tenha definido parâmetros
    DIni = '2024-07-12 10:00:00';        % Indica data hora de inicio apenas para buscar as condilções iniciais
end    
Data_Ini = datetime(DIni);                   % Indica data hora de inicio apenas para buscar as condilções iniciais
DeltaT=1/(24*36);                                % Soma um detla T automático (pouco menos que 2 min)
Data_Fim = Data_Ini+DeltaT;             % Data final apenas para buscar o ponto de inicialização igual ou o mais próximo possível da data indicada
   
%=============================
%NomeArq= 'df_opc_mpa_11_12s.parquet';
NomeArq= 'df_opc_mpa_10s.parquet';

% Carrega dados para condição inicial da planta na faixa de datas pré-selecionadas
[Tt,T]=DadosBase(NomeArq, Data_Ini,Data_Fim);

% Atribui a coniição inicial na data especificada
 YIni=  [Tt.pressao_succao_BCSS(1,1);Tt.pressao_chegada(1,1);Tt.pressao_diferencial_BCSS(1,1);...
        Tt.pressao_descarga_BCSS(1,1);Tt.temperatura_motor_BCSS(1,1);Tt.corrente_torque_BCSS(1,1);...
        Tt.corrente_total_BCSS(1,1);Tt.temperatura_succao_BCSS(1,1);Tt.vibracao_BCSS(1,1);...
        Tt.temperatura_chegada(1,1)];
UIni=[Tt.frequencia_BCSS(1,1);Tt.pressao_montante_alvo(1,1)];
% =========================================
% Proteção para condições iniciais aleatórias
UIni(1)=min(60,UIni(1));      % Garante que a frequencia inicial é igual ou inferior a 60Hz
UIni(1)=max(40,UIni(1));      % Garante que a frequencia inicial é igual ou superior a 40Hz

