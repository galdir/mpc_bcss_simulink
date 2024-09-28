% Exemplo 2
% Usando o CaSAdi para achar um minimo da função Fi(x) = exp(-0.2*x)*sin(x)
% Sujeito a: 0 <= x < = 4pi
%
% Procura a variável de decisão (x) de tal modo que o valor de Fi(x) seja mínimo
%
%
clc
close all
clear all

import casadi.*                              % Importa a biblioteca para definição de expressões matemáticas simbólicas no Casadi

%% Cria o problema não linear (NLP)
x=SX.sym('x');                  % Cria a variável de decisão
obj=exp(0.2*x)*sin(x);    % Cria o objeto correspondente a função

g=[];     % Restrições (sem restrições neste exemplo)
P=[];     % Parâmetros do problema de otimização

OPT_variables=x;      % Variável de decisão

% Define o problema do NLP
nlp_prob=struct('f',obj, 'x',OPT_variables,'g',g,'p',P);

%% Define criterios para a chamada do otimizador CaSAdi da classe nlpsol
opts=struct;                                     % Cria estrutura para conter os critérios
opts.ipopt.max_iter=100;                % Numero máximo de iterações
opts.ipopt.print_level=0;                   % 
opts.print_time=0;                   % 
opts.ipopt.acceptable_tol=1e-8;                   % 
opts.ipopt.acceptable_obj_change_tol=1e-6;                   % 
solver=nlpsol('solver','ipopt',nlp_prob,opts);

%% Para a chamada do otimizador
args=struct;                 % Cria estrutura para compor argumentos do solver
args.lbx=0;                    % Limites inferiores (lower bounds) para a variável de decisão x
args.ubx= 4*pi;               % Limites superiores (upper bounds) para a variável de decisão x
args.lbg=-inf;                % Limites inferiores (lower bounds) para as restrições g
args.ubg= inf;               % Limites superiores (upper bounds) para as restrições g

args.p=[];                      % Sem parâmetros para este problema de otimização

args.x0=1;                 % Condição inicial da variável de decisão x

sol=solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);

x_sol=full(sol.x)

min_value=full(sol.f)









