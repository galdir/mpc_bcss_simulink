function  [sys, x0]  = sfunc(t,x,u,flag,UIni)
% S-Function apenas para a passagem dos valores das manipuladas que foram efetivamente aplicados ao processo
% Isso é necessário pois o controlador enviou commandos que não necessariamente foram atendidos. 
% Assim, o controlador precisa saber em que ponto está o comando atual para poder calcular o DeltaU da frequencia
% e PMonAlvo com referência ao ponto atual e não ao que ele indicou no ciclo anterior. 
% Como dito antes, por proteção, o comando anterior pode não ter sido atendido
%---------------------------------------------------------------------------------------------
% flag = 0 --> Define condições iniciais
if flag == 0
   ninput=2;				% Num de entradas = Comandos de Freq e PMon efetivamente aplicados ao processo 
   nout=2;					% Num. de saida = Comandos de Freq e PMon efetivamente aplicados ao processo
   x0 =UIni;				% Inicialização do vetor de entradas com valores pré-estabelecidos do processo
   sys= [0;size(x0,1); nout; ninput;0;0];  % Depois que defini, nunca mudei desde 199x!
%---------------------------------------------------------------------
% flag = 2 --> retorna estados do sistema
elseif flag == 2
    sys=u;    % Não faz nada senão repassar o que recebeu nas entradas
%---------------------------------------------------------------------
% flag = 3 --> Retorna vetor de saida
elseif flag == 3
	sys= x;				% Retorna o vetor de estados completo
end
