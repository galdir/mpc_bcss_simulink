% Exemplo 3
% Usando o CaSAdi para fazer a função de minimos quadrados y = mx +c
% 
% Assim, coletadas N amostras (x,y), podemos definir o que queremos minimizar
% ficando na froma Fi(m,c) = soma de 1 a N ([ y(i) - (mx(i)+c)]^2)
%
% Procura a variável de decisão (m,c) de tal modo que o valor de Fi seja mínimo
%
%
clc
close all
clear all

rand('seed',1)
randn('seed',2)
import casadi.*                              % Importa a biblioteca para definição de expressões matemáticas simbólicas no Casadi

% Gera dados de uma reta para encontrar os minimos quadrados
N=100;     % 100 amostras
for i=1:N
    x(i)=i;
    y(i)=2.88*x(i)+574;
end
y=awgn(y,10);         % Insere ruido aditivo no sinal com a relação SNR especificada


%% Cria o problema não linear (NLP)
m=SX.sym('m');                  % Cria a variável de decisão
c=SX.sym('c');                    % Cria a variável de decisão

obj=0;      % Inicializa função objetivo
for i=1:N
    obj=obj+( y(i) - (m*x(i)+c))^2;    % Cria o objeto correspondente a função de minimos quadrados
end
    
g=[];     % Restrições (sem restrições neste exemplo)
P=[];     % Parâmetros do problema de otimização

OPT_variables=[m,c];      % Variáveis de decisão

% Define o problema do NLP
nlp_prob=struct('f',obj, 'x',OPT_variables,'g',g,'p',P);

%% Define criterios para a chamada do otimizador CaSAdi da classe nlpsol
opts=struct;                                     % Cria estrutura para conter os critérios
opts.ipopt.max_iter=1000;                % Numero máximo de iterações
opts.ipopt.print_level=0;                   % 
opts.print_time=0;                   % 
opts.ipopt.acceptable_tol=1e-8;                   % 
opts.ipopt.acceptable_obj_change_tol=1e-6;                   % 
solver=nlpsol('solver','ipopt',nlp_prob,opts);

%% Para a chamada do otimizador
args=struct;                 % Cria estrutura para compor argumentos do solver
args.lbx=-inf;                    % Limites inferiores (lower bounds) para as variáveis de decisão
args.ubx= inf;               % Limites superiores (upper bounds) para as variáveis de decisão
args.lbg=-inf;                % Limites inferiores (lower bounds) para as restrições g
args.ubg= inf;               % Limites superiores (upper bounds) para as restrições g

args.p=[];                      % Sem parâmetros para este problema de otimização

args.x0=[0.5, 1 ];              % Condição inicial das variáveis de decisão

sol=solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);

x_sol=full(sol.x);
m_sol=x_sol(1)
c_sol=x_sol(2)

min_value=full(sol.f);

%% Para visualizar a função objetivo
obj_fun=Function('obj_fun',{m,c},{obj});
m_range=-1:0.5:6;
c_range=400:50:800;
obj_plot_data=[];

[mm,cc]=meshgrid(m_range,c_range);

for n=1:1:size(mm,1)
    for k= 1:1:size(mm,2)
        obj_plot_data(n,k)=full(obj_fun(mm(n,k),cc(n,k))); 
    end
end

figure(1)
surf(mm,cc,obj_plot_data);
hold on
xlabel('m')
ylabel('c')
zlabel('\phi')







