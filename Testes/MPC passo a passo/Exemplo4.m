% Exemplo 4
% Usando o CaSAdi para fazer o controle ótimo da movimentação de um robô
% Opção do SingleShooting
%
% 
% Equação dinâmica do sistema a ser controlado
% x_pto=v.cos(Teta);      ou discretizado    x(k+1) =  x(k) + DeltaT*(v.cos(Teta))
% y_pto=v.sin(Teta);                                    y(k+1) =  y(k) + DeltaT*(v.sin(Teta))
% Teta_xpto=w;                                             Teta(k+1) =  Teta(k) + DeltaT*(w) 

% Só para montar o conjunto de equações do Solver
Exemplo4_MontaSolver;
Obstaculo=0;   % Este exemplo não usa obstáculo

t0=0;                            % Inicializa Tempo da amostra 
t(1)=t0;                         % Variável para guardar o vetor de tempo 

x0=[0;0;0];                   % Estados iniciais (x, y,Teta)
xs=[1.5;1.5;pi];            % Estados finais desejados
xx(:,1)=x0;                   % Cria variável para guardar a evolução dos estados

u0=zeros(N,2);         % Inicialização da ação de controle
sim_time=20;            % Tempo de simulação
mpciter=0;                 % Contador para iterações do MPC

xx1=[];
u_cl=[];

while (norm((x0-xs),2) > 1e-2 && mpciter<sim_time/T)
    args.p=[x0;xs];                                % Atualiza parâmetros com o estado atual e o estado final desejado
    args.x0=reshape(u0',2*N,1);          % Formata estados (condição atual e condição desejada) como vetor para passar ao solver
    sol=solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
    u=reshape(full(sol.x)',2,N)';              % Resgata ações de controle para formato matricial
    ff_value=ff(u',args.p);           % Calcula solução com trajetória ótima
    xx1(:,1:3,mpciter+1)=full(ff_value)';
    u_cl=[u_cl;u(1,:)];
    t(mpciter+1)=t0;
    [t0, x0, u0]=shift(T, t0, x0, u, f);
    xx(:,mpciter+2)=x0;
    mpciter=mpciter+1;    
end

PlotaExemplos4a6
