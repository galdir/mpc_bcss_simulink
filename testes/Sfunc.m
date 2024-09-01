function  [sys, x0]  = Sfunc(t,x,u,flag)
if flag == 0
   ninput=1;				% Num de entradas 
   nout=1;					% Entrada Atual
   x0 = [0; 0];				% Inicializa vetor de estados (coluna) Valor atual e valor passado
   sys= [0;size(x0,1); nout; ninput;0;0];  % Depois que defini, nunca mudei desde 199x!
%
%---------------------------------------------------------------------
% flag = 2 --> retorna estados do sistema
elseif flag == 2
sys =[u(1); x(1)+u(1)];		% Entrada e Entrada+Valor passado
%---------------------------------------------------------------------
% flag = 3 --> Retorna vetor de saida
elseif flag == 3
	sys= x(1);				% Retorna o proprio vetor de estados
end
