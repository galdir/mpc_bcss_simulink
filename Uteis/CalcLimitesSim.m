function  [sys, x0]  = CalcLimitesSim(t,x,u,flag,FreqIni,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa)
% Função para calcular as restrições dinâmicas em função da frequência
% Só para ter no Simulink os mesmos cálculos dos limites dinâmicos calculados pela CalcLimites.m
%---------------------------------------------------------------------------------------------
% flag = 0 --> Define condições iniciais
if flag == 0
   ninput=1;			          % Num de entradas = Freq
   Limites=BuscaLimites(FreqIni,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa);
   nout= height(Limites);  % Num. de saida = Limtes H e L das variáveis do processo
   x0 = Limites;    	        % Inicializa vetor de estados (coluna) com limites pré-calculados
   sys= [0;size(x0,1); nout; ninput;0;0];  % Depois que defini, nunca mudei desde 199x! :-)
%--------------------------------------------------------------------------------------------
% flag = 2 --> Retorna estados do sistema
elseif flag == 2
    Freq=u(1);                  % Resgata valor da Frequência na entrada do bloco
    Limites=BuscaLimites(Freq,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa);
    sys=[Limites];		       % Retorna novos limites calculados
    %--------------------------------------------------------------------------------------------
% flag = 3 --> Retorna vetor de saida
elseif flag == 3
	sys= x;				% A saída é o próprio vetor de estados
end

% ===================================================================================
% ============================  FIM DA FUNÇÃO PRINCIPAL ==============================
% ===================================================================================
function Limites=BuscaLimites(Freq,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa)
% Usa a função de cálculo dos limites definida na pasta ÚTEIS
Limites=CalcLimites(Freq,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa);
Limites=Limites(:,2:end);             % Elimina a coluna inicial que indica texto MAX e MIN
Limites=table2array(Limites);      % Transforma tabela em array para poder ser consumida no Simulink
Limites=Limites(:);                        % Contas para transformar em vetor para ser usado no Simulink


