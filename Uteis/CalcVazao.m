function  [sys, x0]  = CalcVazao(t,x,u,flag,MatrizSimulador,DeltaT,QIni)
% Função para calcular a produção de óleo acumulada ao longo da simulação
%---------------------------------------------------------------------------------------------
% flag = 0 --> Define condições iniciais
if flag == 0
   ninput=2;			  % Num de entradas = Freq e PChegada
   nout=3;				   % Num. de saida = Vazão, Produção de Óleo e Produção de Óleo Acumulada
   x0 = [QIni;0;0];		% Inicializa vetor de estados (coluna), no caso, Vazão Inicial, Produção=0 e Produção Acumulada=0
   sys= [0;size(x0,1); nout; ninput;0;0];  % Depois que defini, nunca mudei desde 199x! :-)
%--------------------------------------------------------------------------------------------
% flag = 2 --> Retorna estados do sistema
elseif flag == 2
    Freq=u(1);                                     % Resgata valor da Frequência na entrada do bloco
    PChegada=u(2)*1.019716;         % Resgata valor da P.Chegada [bar] na entrada do bloco e converte para Kgf/cm2 para poder usar a interpolaçao do simulador Petrobras
    ProducaoAcumulada=x(3);          % Resgata valor da produção acumulada
    
    %  Novas contas para saber da producao na janela da tempo da simulação
     % Vazão Instantânea em m3/dia
    Vazao=Interpola(Freq,PChegada,MatrizSimulador,3);  % Busca referência de vazão através da tabela do simulador da Petrobras 

    % Calcula produção em função da vazão estimada
    Producao=Vazao/(24*3600);      % Vazão Instantânea em m3/s
    Producao=Producao*DeltaT;      % Produção (m3) na janela de tempo (Ts padrão = 10s)
    ProducaoAcumulada=ProducaoAcumulada+Producao;   % Incrementa produção acumulada com a produção atual
    sys=[Vazao;Producao;ProducaoAcumulada];		    % Retorna Vazão no instante (m3/d), Produção no instante (m3) e Produção acumulada (m3)
    %--------------------------------------------------------------------------------------------
% flag = 3 --> Retorna vetor de saida
elseif flag == 3
	sys= x;				% A saída é o próprio vetor de estados com a Vazão, Produção e a Produção Acumulada
end
%--------------------------------------------------------------------------------------------


