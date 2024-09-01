function [sys,x0] = calc_ESNx(t,x,u,flag,ESN,YIni)
 nx = 10;  % número dos estados  do processo. Mesma variável consumida em "flags" diferentes
% =========================================================================
% flag = 0 --> Define condições iniciais do processo   
% =========================================================================
if flag == 0
	ninput=2+nx;                  % Número das entradas do processo (2 manipuladas + variáveis à serem preditas)
	nout=nx+1;			           % Número de variáveis do processo + tempo de processamento		 
    x0 = [ YIni; ESN.a0; 0];  % Condição inicial dos estados do reservatório da ESN e tempo de processamento
    sys= [0;size(x0,1);nout;ninput;0;0];  
    
% =========================================================================
% flag = 2 --> Retorna os estados do sistema 
% =========================================================================
elseif abs(flag) == 2
    tStart=tic;                                                       % Marcador para tempo de execução
    xres = x(nx+1:end-1);                                    % Resgata estados do reservatório 
    
    uk=normaliza_entradas(u);
    if (t==0)                                                          % Indica que é a inicialização e usa isso para "esquentar" a ESN
        N=1000;                                                     % N loops (N empírico) para atialização do reservatório 
	    xres=esquenta_ESN(ESN, uk,1000);      % Roda ESN por N loops atualizando os estados do reservatório da ESN
    end
   
    % Calcula novos estados da ESN 
    xk_1 = ESN.Wrr*xres +  ESN.Wir*uk + ESN.Wbr;      % calcula novo estado (xk_1) para uma nova entrada (uk) 
    next_state = (1-ESN.gama)*xres + ESN.gama*tanh(xk_1);
    
    a_wbias = [1.0;next_state];
    xres = next_state;                                                    % Atualiza o estado do reservatório
    yk_aux = ESN.Wro*a_wbias;                                 % Calcula a saída com estado do reserv. atualizado
    yk_1 = desnormaliza_predicoes(yk_aux);             % Resgata unidades de engenharia para simulação
   
    t_execucao = toc(tStart);                                        % Tempo gasto para o processamento em segundos
       
    sys =[yk_1;xres;t_execucao];		                         % Retorna o vetor de estados em coluna

% =========================================================================
% flag = 3 --> Retorna vetor de saida
% =========================================================================
elseif flag == 3
	sys=[x(1:nx) ; x(end)];   % Retorna apenas as novas variáveis estimadas e o tempo de execução  
end
% =========================================================================


   

