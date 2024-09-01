function  [sys, x0]  = sfunc(t,x,u,flag,FreqIni)
% Fun��o para simular a opera��o do MPA no que tange ao controle na
% varia��o m�xima da frequ�ncia
% Varia��o de no m�ximo 1Hz em 7,5min  (450s)
% 
% TamVetor � o tamanho do vetor hist�rico para ser considerado
% No caso espec�fico, considerando Ts=10s, precisa avaliar se na janela de
% 45 amostras (45 * 10s) cabe a varia��o de frequ�ncia proposta, sem violar
% a regra de 1Hz a cada 7,5min

TamVetor=45;       % Tamanho do vetor para armazenar delta a cada passo de 10s  (45x10=450s=7,5min)

%---------------------------------------------------------------------------------------------
% flag = 0 --> Define condi��es iniciais
%

if flag == 0
   ninput=2;			          	   % Num de entradas = 2 = Freq e DeltaFreq
   nout=2;					            % Num. de saida = Freq para opera��o e DeltaFreq  (corrigidos pelo limitador do MPA)
   DeltasPassados=zeros(TamVetor-1,1);  % Menos 1 pois temos o DeltaFreq na entrada atual
   x0 = [FreqIni;0;DeltasPassados];				% Inicializa vetor de estados (coluna): Freq, DeltaFreq e valores passados de DeltaFreq
   sys= [0;size(x0,1); nout; ninput;0;0];         % Depois que defini, nunca mudei desde 199x!
%
%---------------------------------------------------------------------
% flag = 2 --> retorna estados do sistema
elseif flag == 2     
    Freq=u(1);                                      % Valor de Frequencia proposto pelo controlador
%     DeltaFreq=u(1)-x(1);     
    DeltaFreq=u(2);                             % Valor de DeltaFreq proposto pelo controlador
    x=[u(1); DeltaFreq;  x(2:end-1)];   % Atualiza estados com Freq e DeltaFreq propostos e desloca os DeltaFreq anteriores (decarta o mais antigo)
    if DeltaFreq~=0                              % Mas avalia se DeltaFreq n�o viola 1Hz em 450s (7,5min)
        if round(sum(x(2:end)),2)>1      % Soma de DeltaFreq na janela de 450s (45 amostras de 10s) = 7,5min foi maior do que 1Hz
            x(1)=x(1)-x(2);                       % Desfaz a movimenta��o proposta para Freq (retira DeltaFreq que foi colocado pelo controlador)
            x(2)=0;                                    % N�o aplica o passo indicado e assume DeltaFreq=0
        end
    end
    sys=x;
%---------------------------------------------------------------------
% flag = 3 --> Retorna vetor de saida
elseif flag == 3
	sys= x(1:2);	 % Retorna apenas Freq e DeltaFreq
end
%---------------------------------------------------------------------

