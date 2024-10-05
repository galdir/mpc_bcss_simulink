function  [sys, x0]  = CalcVazao(t,x,u,flag,MatrizSimulador,DeltaT,QIni)
% Fun��o para calcular a produ��o de �leo acumulada ao longo da simula��o
%---------------------------------------------------------------------------------------------
% flag = 0 --> Define condi��es iniciais
if flag == 0
   ninput=2;			  % Num de entradas = Freq e PChegada
   nout=3;				   % Num. de saida = Vaz�o, Produ��o de �leo e Produ��o de �leo Acumulada
   x0 = [QIni;0;0];		% Inicializa vetor de estados (coluna), no caso, Vaz�o Inicial, Produ��o=0 e Produ��o Acumulada=0
   sys= [0;size(x0,1); nout; ninput;0;0];  % Depois que defini, nunca mudei desde 199x! :-)
%--------------------------------------------------------------------------------------------
% flag = 2 --> Retorna estados do sistema
elseif flag == 2
    Freq=u(1);                                     % Resgata valor da Frequ�ncia na entrada do bloco
    PChegada=u(2)*1.019716;         % Resgata valor da P.Chegada [bar] na entrada do bloco e converte para Kgf/cm2 para poder usar a interpola�ao do simulador Petrobras
    ProducaoAcumulada=x(3);          % Resgata valor da produ��o acumulada
    
    %  Novas contas para saber da producao na janela da tempo da simula��o
     % Vaz�o Instant�nea em m3/dia
    Vazao=Interpola(Freq,PChegada,MatrizSimulador,3);  % Busca refer�ncia de vaz�o atrav�s da tabela do simulador da Petrobras 

    % Calcula produ��o em fun��o da vaz�o estimada
    Producao=Vazao/(24*3600);      % Vaz�o Instant�nea em m3/s
    Producao=Producao*DeltaT;      % Produ��o (m3) na janela de tempo (Ts padr�o = 10s)
    ProducaoAcumulada=ProducaoAcumulada+Producao;   % Incrementa produ��o acumulada com a produ��o atual
    sys=[Vazao;Producao;ProducaoAcumulada];		    % Retorna Vaz�o no instante (m3/d), Produ��o no instante (m3) e Produ��o acumulada (m3)
    %--------------------------------------------------------------------------------------------
% flag = 3 --> Retorna vetor de saida
elseif flag == 3
	sys= x;				% A sa�da � o pr�prio vetor de estados com a Vaz�o, Produ��o e a Produ��o Acumulada
end
%--------------------------------------------------------------------------------------------


