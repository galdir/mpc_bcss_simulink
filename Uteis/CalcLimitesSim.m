function  [sys, x0]  = CalcLimitesSim(t,x,u,flag,FreqIni,MatrizSimulador,BTP,MatrizRestricoesDinamicas,FxPercent,ProtecaoFixa)
    % Função para calcular as restrições dinâmicas em função da frequência
    % Só para ter no Simulink os mesmos cálculos dos limites dinâmicos calculados pela CalcLimites.m
    %---------------------------------------------------------------------------------------------
    % flag = 0 --> Define condições iniciais
    if flag == 0
       ninput=1;			          % Num de entradas = Freq
       Limites=CalcLimites(FreqIni,MatrizSimulador,BTP,MatrizRestricoesDinamicas,FxPercent,ProtecaoFixa);
       Limites=organiza(Limites);  % Para não perder a formatação dada no Simulink
       nout= height(Limites);  % Num. de saida = Limtes H e L das variáveis do processo
       x0 = Limites;    	        % Inicializa vetor de estados (coluna) com limites pré-calculados
       sys= [0;size(x0,1); nout; ninput;0;0];  % Depois que defini, nunca mudei desde 199x! :-)
    %--------------------------------------------------------------------------------------------
    % flag = 2 --> Retorna estados do sistema
    elseif flag == 2
        Freq=u(1);                  % Resgata valor da Frequência na entrada do bloco
        Limites=CalcLimites(Freq,MatrizSimulador,BTP,MatrizRestricoesDinamicas,FxPercent,ProtecaoFixa);
        Limites=organiza(Limites);
       sys=Limites;		       % Retorna novos limites calculados
        %--------------------------------------------------------------------------------------------
    % flag = 3 --> Retorna vetor de saida
    elseif flag == 3
        sys= x;				% A saída é o próprio vetor de estados
    end
end
%% =====================================================
%% Fim da função principal
function L=organiza(Limites);
% Função criada para não perder a formatação dada no Simulink
    for i=1:width(Limites)
        L(2*i-1)=Limites(1,i);
        L(2*i)=Limites(2,i);
    end
    L=L';
end
%=============================
