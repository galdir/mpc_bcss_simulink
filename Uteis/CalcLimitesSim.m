function  [sys, x0]  = CalcLimitesSim(t,x,u,flag,FreqIni,FuncaoBuscaLimites)
    % Fun��o para calcular as restri��es din�micas em fun��o da frequ�ncia
    % S� para ter no Simulink os mesmos c�lculos dos limites din�micos calculados pela fun��o
    %---------------------------------------------------------------------------------------------
    % flag = 0 --> Define condi��es iniciais
    if flag == 0
       ninput=1;			          % Num de entradas = Freq
       Limites=full(FuncaoBuscaLimites(FreqIni));
       Limites=organiza(Limites);  % Para n�o perder a formata��o dada no Simulink
       nout= height(Limites);  % Num. de saida = Limtes H e L das vari�veis do processo
       x0 = Limites;    	        % Inicializa vetor de estados (coluna) com limites pr�-calculados
       sys= [0;size(x0,1); nout; ninput;0;0];  % Depois que defini, nunca mudei desde 199x! :-)
    %--------------------------------------------------------------------------------------------
    % flag = 2 --> Retorna estados do sistema
    elseif flag == 2
       Freq=u(1);                  % Resgata valor da Frequ�ncia na entrada do bloco
       Limites=full(FuncaoBuscaLimites(Freq));
       Limites=organiza(Limites);  % Para n�o perder a formata��o dada no Simulink
       sys=Limites;		       % Retorna novos limites calculados
        %--------------------------------------------------------------------------------------------
    % flag = 3 --> Retorna vetor de saida
    elseif flag == 3
        sys= x;				% A sa�da � o pr�prio vetor de estados
    end
end
%% =====================================================
%% Fim da fun��o principal
function L=organiza(Limites);
% Fun��o criada para n�o perder a formata��o desejada na sa�da no Simulink (aos pares H/L  H/L e n�o HHHH  LLLLL)
    for i=1:width(Limites)
        L(2*i-1)=Limites(1,i);
        L(2*i)=Limites(2,i);
    end
    L=L';
end
%=============================
