function  [sys, x0]  =ExecutaPlano(t,x,u,flag,Plano)
% Função para oferecer Freq e PMonAlvo de acordo com plano pré-estabelecido
%---------------------------------------------------------------------------------------------
    %% flag = 0 --> Define condições iniciais
    if flag == 0
       ninput=0;			                                                       % Não precisa de entradas
       nout= 2;                                                                     % Freq e PMonAlvo
       x0 = [Plano.Frequencia(1); Plano.PMonAlvo(1)];   % Inicializa vetor de estados (coluna) com Freq e PMonAlvo iniciais do plano
       sys= [0;size(x0,1); nout; ninput;0;0];                        % Depois que defini, nunca mudei desde 199x! :-)
    %--------------------------------------------------------------------------------------------
    %% flag = 2 --> Retorna estados do sistema
    elseif flag == 2
     % Verifica tempo de simulação em relação ao tempo do plano
     VetorTempo=Plano.Tempo;
    [val,pos]=min(t>VetorTempo);   % Busca indice do vetor comparando com tempo de simulação t      
    if ~val   % Se achou mínimo > 0 é porque encerrou e não há mais o que fazer (simulação se mantem no último ponto do plano) 
        x=[Plano.Frequencia(pos); Plano.PMonAlvo(pos)];
    end
    sys=x;		       % Retorna novos valores de Freq. e PmonAlvo definidos no plano
   %--------------------------------------------------------------------------------------------
    %% flag = 3 --> Retorna vetor de saida
    elseif flag == 3
        sys= x;				% A saída é o próprio vetor de estados
    end
    
end

