% Monta Solver que será utilizado no Exemplo 5
% Usando o CaSAdi para fazer o controle ótimo da movimentação de um robô
% Opção do MultipleShooting
% 
% Equação dinâmica do sistema a ser controlado
% x_pto=v.cos(Teta);      ou discretizado    x(k+1) =  x(k) + DeltaT*(v.cos(Teta))
% y_pto=v.sin(Teta);                                      y(k+1) =  y(k) + DeltaT*(v.sin(Teta))
% Teta_xpto=w;                                             Teta(k+1) =  Teta(k) + DeltaT*(w) 

clc
close all
clear all
import casadi.*                              % Importa a biblioteca para definição de expressões matemáticas simbólicas no Casadi

T=0.2;                 % Período de amostragem
N=10;                    % Horizonte de predição
Q=diag([1  5  0.1]);     % Matriz diagonal de pesos dos estados
R=diag([0.5  0.05]);    % Matriz diagonal de pesos das ações de controle (manipuladas)

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
X=SX.sym('X',n_states,(N+1));                  % Matriz de estados empilhando o problema de otimização

obj=0;
g=[];        % Inicializa nula

% Montagem das restrições (atuais e futuras)
st=X(:,1);              % Condição inicial dos estados
g=[g;st-P(1:3)];    % Restrição das condições iniciais
for k=1:N
    st=X(:,k);
    con=U(:,k);
    obj=obj + (st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con; 
    st_next=X(:,k+1);
    f_value=f(st,con);
    st_next_euler=st+(T*f_value);
    g=[g  ;  st_next - st_next_euler ];
end



% Monta as variáveis de decisão em um vetor coluna
OPT_variables=[reshape(X,3*(N+1),1)  ;   reshape(U,2*N,1)];

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

% restrições
args.lbg(1:3*(N+1))=0;                % Limites inferiores (lower bounds) para as restrições de igualdade em g
args.ubg(1:3*(N+1))=0;               % Limites superiores (upper bounds) para as restrições de igualdade em g

args.lbx(1:3:3*(N+1))=-2;
args.ubx(1:3:3*(N+1))=2;
args.lbx(2:3:3*(N+1))=-2;
args.ubx(2:3:3*(N+1))=2;
args.lbx(3:3:3*(N+1))=-inf;
args.ubx(3:3:3*(N+1))=inf;

args.lbx(3*(N+1)+1:2:3*(N+1)+2*N)=v_min;                    % Limites inferiores (lower bounds) para as variáveis de decisão de v
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N)=v_max;               % Limites superiores (upper bounds) para as variáveis de decisão de v

args.lbx(3*(N+1)+2:2:3*(N+1)+2*N)=omega_min;               % Limites inferiores (lower bounds) para as variáveis de decisão de omega
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N)=omega_max;          % Limites superiores (upper bounds) para as variáveis de decisão de omega


disp('Montado o solver para o Exemplo 5')