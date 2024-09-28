% Monta Solver que será utilizado no Exemplo 4
% Usando o CaSAdi para fazer o controle ótimo da movimentação de um robô
% Opção do SingleShooting
% 
% Equação dinâmica do sistema a ser controlado
% x_pto=v.cos(Teta);      ou discretizado    x(k+1) =  x(k) + DeltaT*(v.cos(Teta))
% y_pto=v.sin(Teta);                                     y(k+1) =  y(k) + DeltaT*(v.sin(Teta))
% Teta_xpto=w;                                             Teta(k+1) =  Teta(k) + DeltaT*(w) 

clc
close all
clear all
import casadi.*                              % Importa a biblioteca para definição de expressões matemáticas simbólicas no Casadi

T=0.2;                 % Período de amostragem
N=10;                    % Horizonte de predição

v_max=0.6;   
v_min=-v_max;
omega_max=pi/4;
omega_min=-omega_max;

%% Cria o problema não linear (NLP)
% Estados do processo controlado
x=SX.sym('x');                                % Cria a variável de decisão
y=SX.sym('y');                                 % Cria a variável de decisão
theta=SX.sym('theta');                    % Cria a variável de decisão

states=[x;y;theta];                           % Variável para guardar estados do processo
n_states=length(states);                  % Variável para guardar o número de estados

% Variáveis manipuladas
v=SX.sym('v');                                 % Cria a variável de decisão
omega=SX.sym('omega');              % Cria a variável de decisão

controls=[v;omega];
n_controls=length(controls);                  % Variável para guardar o número de variáveis manipuladas

rhs=[v*cos(theta); v*sin(theta); omega];  % Sistema dinâmico

f=Function('f',{states controls},{rhs});       % Função não linear f(x,u)
U=SX.sym('U',n_controls,N);                    % Variáveis de decisão (ações de controle atuais e futuras até o horizonte de predição)
P=SX.sym('P',n_states+n_states);           % Parametros com as consições iniciais do sistema e posição de referência do robô
X=SX.sym('X',n_states,(N+1));                 % Matriz de estados empilhando o problema de otimização

% Cálculo da solução com variáveis simbólicas
X(:,1)=P(1:3);
for k=1:N
    st=X(:,k);                                             % Estados atuais do sistema
    con=U(:,k);                                         % Ação de controle atual no sistema
    f_value=f(st,con);                               % Valor da saida quando aplicada a ação de controle no estado atual
    st_next=st + (T*f_value);                   % Calcula o estado futuro do sistema
    X(:,k+1)=st_next;                                 % Atualiza o estado do sistema para ser considerado no próximo passo do loop 
end

%% Função que compõe a trajetória ótima calculada
ff=Function('ff',{U,P},{X});

Q=diag([1  5  0.1]);     % Matriz diagonal de pesos dos estados
R=diag([0.5  0.05]);    % Matriz diagonal de pesos das ações de controle (manipuladas)

% Montagem da função objetivo
obj=0;          % Inicialização com zero
for k=1:N     % Varre de k até o horizonte de predição
    st=X(:,k);
    con=U(:,k);
    obj=obj + (st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con; 
end

% Montagem das restrições (atuais e futuras)
g=[];        % Inicializa nula
for k=1:N+1
        g=[g;X(1,k)];    % Estados de x
        g=[g;X(2,k)];    % Estados de y
end

% Monta as variáveis de decisão em um vetor coluna
OPT_variables=reshape(U,2*N,1);

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

args.lbx(1:2:2*N-1)=v_min;                    % Limites inferiores (lower bounds) para as variáveis de decisão de v
args.lbx(2:2:2*N)=omega_min;               % Limites inferiores (lower bounds) para as variáveis de decisão de omega

args.ubx(1:2:2*N-1)=v_max;               % Limites superiores (upper bounds) para as variáveis de decisão de v
args.ubx(2:2:2*N)=omega_max;          % Limites superiores (upper bounds) para as variáveis de decisão de omega

% restrições de desigualdade
args.lbg=-2;                % Limites inferiores (lower bounds) para as restrições g
args.ubg= 2;               % Limites superiores (upper bounds) para as restrições g

disp('Montado o solver para o Exemplo 4')