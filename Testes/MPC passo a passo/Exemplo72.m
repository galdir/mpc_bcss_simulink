% Monta Solver que será utilizado no Exemplo 7
% Usando o CaSAdi para fazer o controle ótimo da movimentação de um robô
% Opção do MultipleShooting
% Exemplo ajustado o código para empilhar restrições dentro do mesmo laço
% de horizonte predito, ou seja, monta as restrições e em sequencia os
% respectivos limites, favorecendo o entendimento da indexação

% Neste caso, fazemos o limite máximo da velocidade linear variar em função da posição x
% As restrições dinâmicas da velocidade foram inseridas em lbg/ubg
% Isso, pois, para os estados do MPC (X e U), só é possivel definir limites
% e por restrições do SOLVER, não é possível inserir fórmulas nos limites. Os limites sempre devem ser fixos pré estabelecidos.
% Assim, para inserir restrições em g, geramos uma fórmula (usualmente uma igualdade ou uma diferença) e 
% fazemos os limites fixos nas chamadas restrições de igualdade ou desigualdade, onde os limites são fixos/conhecidos
% Assim, para aplicar, a fórmula para o g deve tratar a igualdade/desigualdade desejada e lbg/ubg os respectivos limites

% Equação dinâmica do sistema a ser controlado
% x_pto=v.cos(Teta);      ou discretizado    x(k+1) =  x(k) + DeltaT*(v.cos(Teta))
% y_pto=v.sin(Teta);                                      y(k+1) =  y(k) + DeltaT*(v.sin(Teta))
% Teta_xpto=w;                                             Teta(k+1) =  Teta(k) + DeltaT*(w) 

% ONDE:
% x_pto é a posição na coordenada X do robô
% y_pto é a posição na coordenada Y do robô
% Teta = orientação em que o robô está (Ângulo em radianos)

% Nestes exemplos, a ideia é controlar os estados (x_pto, y_pto e Teta)
% alterando V e W (vlocidade linear e velocidade angular)

clc
close all
clear all

import casadi.*                              % Importa a biblioteca para definição de expressões matemáticas simbólicas no Casadi

%% =======================================
%% Definições do usuário

% Dados para a simulação
T=0.2;                         % Período de amostragem
Obstaculo=1;               % (1/0) Este exemplo usa obstáculo
x0=[-2;-2;0];                 % Estados iniciais (x, y,Teta)
xs=[1.8;1.8;-pi];             % Estados finais desejados
sim_time=30;              % Tempo de simulação em s (Amostras = sim_time/T)

% Para a sintonia do MPC
N=10;                          % Horizonte de predição
Q=diag([1  5  0.1]);     % Matriz diagonal de pesos dos estados
R=diag([0.5  0.05]);    % Matriz diagonal de pesos das ações de controle (manipuladas)

% Referências do raio do robô para caracterizar espaço de evitar colisão
% (se desabilitar a restrição dinâmica [g] que varia em função deste raio, o Solver
% assumirá este valor e mostrará que pelo caminho percorrido iria coliir)
robo_r=0.3;    % Este valor foi usado nos exemplos 4 a 6 mas neste exemplo será alterado em função das restrições dinâmicas

% Referências do obstáculo à ser evitado
obs_x=0.5;   obs_y=0.5;   obs_r=0.5;
Obstacle=[obs_x   obs_y   obs_r];   % Coordenadas (x,y) do obstáculo com tamanho (raio) do obstáculo

% Limites (bounds) fixos para os 3 estados, as quais valerão para todo o horizonte predito
FixedStatesLowerBounds= [-3 -3   -inf];       % Xrobo >- 2    Yrobo > -2     Orientação Teta  > -inf
FixedStatesUpperBounds= [ 3   3   inf];         % Xrobo < 2     Yrobo < 2      Orientação Teta < inf
ZeroBounds=[ 0  0  0];     % Para quando precisarmos de limite de igualdade para todos os estados

% Limites para as duas ações de controle, as quais valerão para todo o horizonte predito
v_max=0.6;                            % Valor máximo para a velocidade linear V (manipulada1)
v_min=0.1;                      % Valor mínimo para a velocidade linear V (manipulada1)
omega_max=pi/4;                  % Valor mínimo para a velocidade algular W (manipulada2)
omega_min=-omega_max;   % Valor mínimo para a velocidade angular W (manipulada2)
ControlActionLowerBound= [ -v_max   omega_min];   % Valores mínimos para compor limites nas ações de controle
ControlActionUpperBound= [ v_max   omega_max]; % Valores máximos para compor limites nas ações de controle

%% ==============================================
%% Prepara o ambiente simbólico para o problema não linear (NLP)
% Formulação do sistema RHS a ser controlado
x=SX.sym('x');                                % Cria a variável de decisão (primeiro estado do processo)
y=SX.sym('y');                                 % Cria a variável de decisão (segundo estado do processo)
theta=SX.sym('theta');                    % Cria a variável de decisão (terceiro estado do processo)

states=[x;y;theta];                           % Variável para guardar estados do processo
n_states=length(states);                  % Variável para guardar o número de estados

% Variáveis manipuladas (velocidade linear V e velocidade angular Ômega)
v=SX.sym('v');                                 % Cria a variável de decisão
omega=SX.sym('omega');              % Cria a variável de decisão
vMax=SX.sym('vMax');                   % Cria limite para a variável de decisão (que será dinâmico)

controls=[v;omega];
n_controls=length(controls);                  % Variável para guardar o número de variáveis manipuladas

rhs=[v*cos(theta); v*sin(theta); omega];  % Sistema dinâmico (vindo da equação de estados do problema)

f=Function('f',{states controls},{rhs});       % Função não linear f(x,u), fornece o resultado das entradas {estados e ações de controle} no sistema dinâmico rhs
U=SX.sym('U',n_controls,N);                    % Variáveis de decisão (ações de controle atuais e futuras até o horizonte de predição = N)
P=SX.sym('P',n_states+n_states);           % Parametros que vão  conter os estados medidos em k (1:3) e os estados desejados no k+1 (4:6). Definidos em args.p
X=SX.sym('X',n_states,(1+N));                  % Matriz de estados com n_states colunas e linhas = 1+horizonte de predição

%% ===================================================
% Montagem dos limites (atuais e futuros) dos estados X
args=struct;     % Inicializa variável que vai armazenar a estrutura de argumentos

% Inicializa variável para guardar os limites dos estados X e ações de controle U
% Observe que necessariamente terá a forma de [ n_states  n_control], e o que vai acontecer no loop é empilhar
% este mesmo formato para os valores futuros preditos para todo o horizonte Hp
args.lbx=[];       % Inicializa limites inferiores para os estados X do MPC e ações de controle
args.ubx=[];     % Inicializa limites superiores para os estados X do MPC e ações de controle

% Inicializa variável para guardar as fórmulas das restrições que vamos criar livremente
% Neste caso, o formato que for criado também deve ser considerado para os valores em todo o horizonte de predição
g=[];             
args.lbg=[];       % Inicializa limites inferiores para as restrições que vamos criar "livremente"
args.ubg=[];     % Inicializa limites superiores para as restrições que vamos criar "livremente"

st=X(:,1);                     % Inicializa variável com estados atuais X0 do sistema (observe que X é uma variável simbólica)
args.lbx=[args.lbx, FixedStatesLowerBounds];  % Incrementa limites inferiores para as restrições de X na condição atual      
args.ubx=[args.ubx,FixedStatesUpperBounds]; % Incrementa limites suferiores para as restrições de X na condição atual

g=[g;st-P(1:n_states)];   % Restrição como a diferença entre estados atuais medidos e a condição inicial dos estados enviados por parâmetro    
% Sendo uma diferença que idealmente deve ser zero, é uma restrição dita como sendo uma restrição de igualdade
% pois os limites inferiores e superiores devem ser igual a zero, já que a busca é pela diferença ser nula
args.lbg=[args.lbg     ZeroBounds];     % Zeros para os limites inferiores  
args.ubg=[args.ubg  ZeroBounds];     % Zeros para os limites superiores, caracterizando a restrição de igualdade (igual a zero)

% Nas últimas linhas anteriores criamos os valores para os estados atuais o sistema (x,y,Teta). 
% Agora o loop vai inserir as restrições para todo o horizonte de predição
obj=0;               % Inicializa custo da função objetivo
for k=1:N                       % Insere restrições para todo o horizonte de predição (contas feitas em variável simbólica para passar ao Solver)
    st=X(:,k);                    % Estados atuais do sistema (x, y, Teta)
    con=U(:,k);                 % Ações de controle atual (v e omega)
    obj=obj + (st-P(n_states+1:end))'*Q*(st-P(n_states+1:end)) + con'*R*con;    % Função objetivo do MPC que deve ser minimizada
%    obj=obj + (st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con;    % Função objetivo do MPC que deve ser minimizada
    % Observar que P(4:6) é o valor do alvo desejado (xs) que veio para o solver na forma de parâmetro
    st_next=X(:,k+1);       % Atualiza os estados com o passo seguinte 
    f_value=f(st,con);      % Aplica a função dinâmica que calcula a saida para o passo seguinte
    st_next_euler=st+(T*f_value);   % Estima um valor futuro com base no método de Euler
%     st_next_euler=st_next_euler+randn/100;   % Estima um valor futuro com base no método de Euler (e soma ruido)
    args.lbx=[args.lbx, FixedStatesLowerBounds];      % Empilha limites inferiores para as restrições do estado 
    args.ubx=[args.ubx, FixedStatesUpperBounds];   % Empilha limites superiores para as restrições do estado
    
    % Criando uma restrição para extra para o erro de predição, forçando com que o robô siga o caminho predito
    ErroPredicao= st_next_euler- st_next ;   % Define erro como sendo a diferença entre o estado predito e o atual (desejado e estimado?)
    g=[g ; ErroPredicao ];                                % Empilha as  restrições da predicao            
    args.lbg = [args.lbg, ZeroBounds];            % Limites inferiores (lower bounds) para os erros de predição igual a zero
    args.ubg = [args.ubg, ZeroBounds ];        % Limites superiores (upper bounds) para os erros de predição igual a zero
end

% Insere restrições futuras referentes as ações de controle (completa a
% forma de LBX e UBX com a parte das ações de controle)
% Lembrando que:
% ControlActionLowerBound= [ v_min   omega_min];   % Valores mínimos para compor restrições nas ações de controle
% ControlActionUpperBound= [ v_max   omega_max]; % Valores máximos para compor restrições nas ações de controle

% Montando as restrições para as velocidades lineares e angulares
for  k=1:N
    % Mantém na faixa antes estabelecida, a região de busca nos estados U do MPC  
    args.lbx=[args.lbx, ControlActionLowerBound];
    args.ubx=[args.ubx, ControlActionUpperBound];

    % Inserindo restrições que variam no tempo
    % Como exemplo, faremos v_max mudar em função da posição X, lembrando que v_min = -v_max 
    % No exemplo, a velocidade pode ser tão maior quanto for o valor da posição x
    st=X(:,k);                    % Estados atuais do sistema (x, y, Teta)
    % Uso limites pré-estabelecidos do problema para criar relação
    % linear que faz a v_max variar do valor original até uma parte dela, inferior ao valor original
    % Gero uma relação linear em função dos valores pré-estabelecidos
    LimMax=v_max;           
    LimMin=v_max/5;
    XVmax=2;                        
    XVmin=-2;
    % Faz contas para saber da relação linear proposta para encontrar v_max como função da posição
    DeltaLim=LimMax-LimMin;
    DeltaX=XVmax-XVmin;
    a=DeltaLim/DeltaX;
    b=(LimMax - a * XVmax);
    vMax=a*st(1)+b;                        % Para ser emplilhada, precisa ser uma variável simbólica (definimos antes)

    con=U(:,k);                              % Ações de controle atuais (v e omega)
%     LimiteAcao=[vMax ;  omega_max]-abs(con);  % Em toda a faixa de -vMax até vMax  diferença terá de ser sempre positiva
%     g=[g ; LimiteAcao ];                              % Empilha as restrições para as ações de controle
%     args.lbg=[args.lbg, [ 0  0] ];                   % A diferença (LimiteAcao tem de ser sempre positiva)
%     args.ubg=[args.ubg,[ inf    inf ]];

%     g=[g ; vMax - abs(con(1)) ];                              % Empilha as restrições para as ações de controle
%     args.lbg=[args.lbg, 0 ];                   % A diferença (LimiteAcao tem de ser sempre positiva)
%     args.ubg=[args.ubg, inf];
% 
%     g=[g ; omega_max - abs(con(2)) ];                              % Empilha as restrições para as ações de controle
%     args.lbg=[args.lbg, 0 ];                   % A diferença (LimiteAcao tem de ser sempre positiva)
%     args.ubg=[args.ubg, inf];

    v_atual = con(1);
    %g=[g; (v_max > abs(v_atual) > v_min) | v_atual==0  ];
    g=[g;  abs(v_atual) > v_min];
    args.lbg=[args.lbg, 1 ];                   % A diferença (LimiteAcao tem de ser sempre positiva)
    args.ubg=[args.ubg, 1 ];
  

end

% Insere restrições atuais e futuras referentes ao obstáculo
for k=1:N+1     
    robo_r=RaioProtecaoDinamica(X(1,k),X(2,k));      % Apenas inserir a fórmula do cálculo do raio em função da coordenada
    Posicao=[ X(1,k)   X(2,k)];                                         % Coordenadas da posição do robô
    Obstacle=[obs_x  obs_y];                                          % Coordenadas do obstáculo
    DistanciaMinEntreCentros= robo_r+obs_r;             % A soma dos dois raios é a distancia minima necessária entre os centros para não haver colisão
    DistanciaAtual=sqrt((Posicao(1)-Obstacle(1))^2 + (Posicao(2)-Obstacle(2))^2);   % Distância euclidiana entre os centros
    Diferenca=DistanciaAtual-DistanciaMinEntreCentros; % Diferença da diatância atual em relação a distância minima      
    g=[g; Diferenca];           % Insere restrição de desigualdade já que a diferença tem de ser sempre maior ou igual a zero
    % E as respectivas restrições precisam assegurar Distancia >=0
    args.lbg=[args.lbg, 0]; % Limites inferiores (lower bounds) para as restrições de desigualdade em g 
    args.ubg=[args.ubg, inf]; % Limites superiores (upper bounds) para as restrições de desigualdade em g
end

%% ===========================================================
%% Para a chamada do otimizador

% Monta as variáveis de decisão em um vetor coluna - esta dimensão é fundamental para entender as contas e  indexações
% Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo HP  +  Ações de controle em todo Hpc - neste caso, Hc=Hp ]
%                             Dimensão = [                              n_states*(N+1)                    ;        n_controls*N ]
OPT_variables=[reshape(X,n_states*(N+1),1)  ;   reshape(U,n_controls*N,1)];

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

disp('Montado o solver para o Exemplo 72')

%% Para o loop de simulação com o MPC
t0=0;                                  % Inicializa Tempo da amostra 
t(1)=t0;                              % Variável para guardar o vetor de tempo 
u0=zeros(N,2);                 % Inicialização da ação de controle
X0=repmat(x0,1,N+1)';    % Inicializa os estados (x,y,Teta) atuais e em todo o horizonte de predição
mpciter=0;                         % Contador para iterações do MPC

xx1=[];                                % Variável para guardar estado atual e estados futuros (para plot)         
u_cl=[];                               % Variável para guardar as ações de controle atuais (para plot)
F=[];                                    % Variável para guardar a Feasability (solução realizável pelo Solver)

while (norm((x0-xs),2) > 1e-2 && mpciter<sim_time/T)
    args.p=[x0;xs];                                % Atualiza parâmetros com o estado atual e o estado final desejado
    args.x0=[ reshape(X0',n_states*(N+1),1);   reshape(u0',n_controls*N,1)];          % Formata estados (condição atual e condição desejada) como vetor para passar ao solver
    sol=solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
    disp(strcat("Feasability: ",num2str(solver.stats.success)));
    Solucao=full(sol.x);    % Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo HP  +  Ações de controle em todo Hp ]
    
    % Extraindo dados da resposta do solver (transpostos são importantes - sugiro não alterar a estrutura padrão do reshape)
    EstadosAtuais=Solucao(1:n_states)';
    EstadosAtuais=reshape(EstadosAtuais,n_states,1)';                     % Formata em linhas
    EstadosFuturos=Solucao(n_states+1:n_states+N*n_states)';
    EstadosFuturos=reshape(EstadosFuturos,n_states,N)';              % Cada linha representará um horizonte
    AcoesControle=Solucao(n_states*(N+1)+1:end)';
    AcoesControle=reshape(AcoesControle,n_controls,N)';               % Cada linha representará um horizonte
    
    u=AcoesControle;
    u_cl=[u_cl;u(1,:)];                                    % Guarda apenas a primeira ação de controle para plotar as ações que foram aplicadas
    
    X0=[EstadosAtuais;EstadosFuturos];   % Estado atual e estados futuros em todo o horizonte
    xx1(:,1:n_states,mpciter+1)=X0;            % Guarda estado atual e estados preditos para o plot
    
    % Atualiza os dados para do loop de simulação
    t(mpciter+1)=t0;                           % Atualiza o tempo de referência para a próxima iteração do MPC
    [t0, x0, u0]=shift(T, t0, x0, u, f);   % Atualiza estados do sistema dinâmico com base no estado atual e no DeltaT
    X0=[X0(2:end,:); X0(end,:)];        % Atualiza estados e como não conhece além do Horizonte, repete o último conhecido
    mpciter=mpciter+1;                       % Incrementa contados de iteração do MPC
end

PlotaExemplos
