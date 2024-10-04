TabRestricoesDinamicas=readtable('DP.xlsx');              % Tabela de valores de referência para as variáveis
FxPercent=readtable('DP-Faixas.xlsx');                           % Tabela das faixas percentuais para disparar os alarmes LL, L, H e HH
BTP=readtable('DoBTP.xlsx');                                           % Tabela de dados do teste de produção
ProtecaoFixa =readtable('FixedProtections.xlsx');            % Carrega tabelas de proteção fixa da Petrobras
TabSimulador=LeConverteNomes('DoSimulador.xlsx');  % Tabela do simulador (Análise de Sensibilidade)


% Só para captar a estrutura da tabela de restrições e inserir a  informação da frequência
TabelaLimites=CalcLimites(40,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa);
TabelaLimites.('Frequencia')=[40;40];     % Cria a coluna com a informação de frequência
TabelaLimites = movevars(TabelaLimites,"Frequencia",'before',"LIMITES");

for Freq=40.1:0.1:60 
    T=CalcLimites(Freq,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa);
    T.('Frequencia')=[round(Freq,1);round(Freq,1)];     % Cria a coluna com a informação de frequência
    % OBS: este ROUND parece não fazer sentido, mas ao criar a tabela sem
    % ele, as vezes o Freq guardava na XLSX uma proximação na enésima casa decimal e gerava problemas !!!
    TabelaLimites=vertcat(TabelaLimites,T);     % Insere novos dados na tabela
end

% Extrai apenas as colunas de interesse
%NomeVariaveis={'Frequencia','LIMITES','TotalCurrent', 'TorqueCurrent', 'IntakePressure', 'DischargePressure',...
%    'DifferentialPressure', 'ProductionSurfacePressure', 'ProductionSurfaceTemperature', 'TMotor', 'Tsuc', 'Vibracao','VazaoOleo'};
NomeVariaveis={'Frequencia','LIMITES','IntakePressure', 'ProductionSurfacePressure', 'DifferentialPressure',...
    'DischargePressure', 'TMotor', 'TorqueCurrent', 'TotalCurrent', 'Tsuc',  ...
     'Vibracao', 'ProductionSurfaceTemperature', 'VazaoOleo'};

TabelaLimites=TabelaLimites(:,NomeVariaveis);   
    
writetable(TabelaLimites,'TabelaLimitesDinamicos.xlsx');
   
