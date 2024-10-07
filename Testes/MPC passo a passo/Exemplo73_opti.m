% Monta Solver que será utilizado no Exemplo 7
% Usando o CaSADi com Opti Stack para fazer o controle ótimo da movimentação de um robô
% Opção do MultipleShooting

clear all
close all
clc

import casadi.*

%% Definições do usuário
T = 0.2;                   % Período de amostragem
Obstaculo = 1;             % (1/0) Este exemplo usa obstáculo
x0 = [-2; -2; 0];          % Estados iniciais (x, y, Teta)
xs = [1.8; 1.8; pi];       % Estados finais desejados
sim_time = 20;             % Tempo de simulação em s (Amostras = sim_time/T)

% Para a sintonia do MPC
N = 15;                    % Horizonte de predição
Q = diag([1 5 0.1]);       % Matriz diagonal de pesos dos estados
R = diag([0.5 0.05]);      % Matriz diagonal de pesos das ações de controle (manipuladas)

% Referências do raio do robô para caracterizar espaço de evitar colisão
robo_r = 0.3;

% Referências do obstáculo à ser evitado
obs_x = 0.5; obs_y = 0.5; obs_r = 0.5;
Obstacle = [obs_x obs_y obs_r];

% Restrições fixas para os 3 estados
FixedStatesLowerBounds = [-10 -10 -inf];
FixedStatesUpperBounds = [10 10 inf];

% Restrições para as duas ações de controle
v_max = 0.6;
v_min = 0.01;
omega_max = pi/4;
ControlActionLowerBound = [-v_max -omega_max];
ControlActionUpperBound = [v_max omega_max];

%% Preparação do problema de otimização usando Opti
opti = casadi.Opti();

% Variáveis de decisão
X = opti.variable(3, N+1);  % Estados para todo o horizonte
U = opti.variable(2, N);    % Ações de controle para todo o horizonte

% Parâmetros
P = opti.parameter(6);      % [x0; xs]

% Função objetivo
obj = 0;
for k = 1:N
    obj = obj + (X(:,k)-P(4:6))'*Q*(X(:,k)-P(4:6)) + U(:,k)'*R*U(:,k);
end
opti.minimize(obj);

% Restrições dinâmicas
for k = 1:N
    x_next = X(:,k) + T*[U(1,k)*cos(X(3,k)); U(1,k)*sin(X(3,k)); U(2,k)];
    opti.subject_to(X(:,k+1) == x_next);
end

% Restrições de estados
opti.subject_to(FixedStatesLowerBounds' <= X <= FixedStatesUpperBounds');

% Restrições de ações de controle
opti.subject_to(ControlActionLowerBound' <= U <= ControlActionUpperBound');

% Restrições de velocidade
for k = 1:N
    v = U(1,k);
    opti.subject_to(v*(v_min-v) <= 0);
    opti.subject_to(-v*(v_min+v) <= 0);
end

% Restrição de obstáculo
for k = 1:N+1
    robo_r = RaioProtecaoDinamica(X(1,k), X(2,k));
    dist = sqrt((X(1,k)-obs_x)^2 + (X(2,k)-obs_y)^2);
    opti.subject_to(dist >= robo_r + obs_r);
end

% Condição inicial
opti.subject_to(X(:,1) == P(1:3));

% Configurações do solver
opti.solver('ipopt');

%% Loop de simulação com o MPC
t0 = 0;
t = [t0];
x0_sim = x0;
mpciter = 0;
xx1 = [];
u_cl = [];

while (norm(x0_sim-xs,2) > 1e-3 && mpciter < sim_time/T)
    % Resolver o problema de otimização
    opti.set_value(P, [x0_sim; xs]);
    sol = opti.solve();
    
    % Extrair resultados
    X_opt = sol.value(X);
    U_opt = sol.value(U);
    
    % Aplicar primeira ação de controle
    u = U_opt(:,1);
    u_cl = [u_cl; u'];
    
    % Atualizar estado
    x0_sim = full(X_opt(:,2));
    
    % Armazenar resultados
    xx1 = [xx1 X_opt];
    t = [t t0];
    
    % Preparar para próxima iteração
    t0 = t0 + T;
    mpciter = mpciter + 1;
end

%% Plotagem dos resultados
PlotaExemplos

function r = RaioProtecaoDinamica(x, y)
    % Implemente a função de cálculo do raio de proteção dinâmica aqui
    r = 0.3; % Exemplo simples, substitua pela lógica real
end