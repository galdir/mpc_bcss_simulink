function  [sys, x0]  = CalcLimitesSim(t,x,u,flag,FreqIni,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa)
% Fun��o para calcular as restri��es din�micas em fun��o da frequ�ncia
% S� para ter no Simulink os mesmos c�lculos dos limites din�micos calculados pela CalcLimites.m
%---------------------------------------------------------------------------------------------
% flag = 0 --> Define condi��es iniciais
if flag == 0
   ninput=1;			          % Num de entradas = Freq
   Limites=BuscaLimites(FreqIni,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa);
   nout= height(Limites);  % Num. de saida = Limtes H e L das vari�veis do processo
   x0 = Limites;    	        % Inicializa vetor de estados (coluna) com limites pr�-calculados
   sys= [0;size(x0,1); nout; ninput;0;0];  % Depois que defini, nunca mudei desde 199x! :-)
%--------------------------------------------------------------------------------------------
% flag = 2 --> Retorna estados do sistema
elseif flag == 2
    Freq=u(1);                  % Resgata valor da Frequ�ncia na entrada do bloco
    Limites=BuscaLimites(Freq,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa);
    sys=[Limites];		       % Retorna novos limites calculados
    %--------------------------------------------------------------------------------------------
% flag = 3 --> Retorna vetor de saida
elseif flag == 3
	sys= x;				% A sa�da � o pr�prio vetor de estados
end

% ===================================================================================
% ============================  FIM DA FUN��O PRINCIPAL ==============================
% ===================================================================================
function Limites=BuscaLimites(Freq,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa)
% Usa a fun��o de c�lculo dos limites definida na pasta �TEIS
Limites=CalcLimites(Freq,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa);
Limites=Limites(:,2:end);             % Elimina a coluna inicial que indica texto MAX e MIN
Limites=table2array(Limites);      % Transforma tabela em array para poder ser consumida no Simulink
Limites=Limites(:);                        % Contas para transformar em vetor para ser usado no Simulink


