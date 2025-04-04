function [solver, args] = cria_solver(umax, umin, dumax, MargemPercentual, ...
        Hp, Hc, Qy, Qu, R, Qx, nx, nu, ny, EstimaVazao, buscaLimites, ModeloPreditor, funcao_h, WallTime)

    % Marcador de tempo e importação de CasADi
    tInicializa = tic;
    import casadi.*;

    nx_ESN = length(ModeloPreditor.data.a0);    
    
    
    %% Variáveis simbolicas para o problema de otimização
    % Para melhor entendimento, as COLUNAS representarão a evolução do instantes k das variáveis de decisão
    X = MX.sym('X',nx,1+Hp);                      % Estado atual + Estados futuros até Hp
    U = MX.sym('U',nu,Hp);                         % Ações de controle até o horizonte Hp, mas para função custo usaremos até Hc
    DU = MX.sym('DU',nu,Hp);                   % Variações nas ações de controle sobre o horizonte Hp, mas para função custo usaremos até Hc-1

    %% ===================================================
    % Parâmetros que foram oferecidos para o Solver
    Indice = [nx ... % qtd Medições
        nu ... % qtd Ações
        nu ... % qtd AlvoEng
        ny ... % qtd Ysp
        nx ... % qtd ErroX
        ny ... % qtd ErroY
        45 ... % tamanho do Buffer de DeltaFreq
        nx_ESN]; % tamanho do estado do modelo de predicao

    % Cria vetor de parâmetros na dimensão especificada
    P = MX.sym('P',sum(Indice));
    
    % Associa variáveis simbólicas as respectivas partes no vetor de parâmetros
    [Xk0, Uk0, AlvoEng, Ysp, ErroX, ErroY, BuffDeltaFreq, Reservatorio_ESN] = ExtraiParametros(P, Indice);

    ModeloPreditor_estado_atual=Reservatorio_ESN;      % Resgata estado atual do reservatório da ESN

    %% ===================================================
    % MONTAGEM DAS RESTRIÇÕES (atuais e futuras) DAS VARIÁVEIS DE DECISÃO (lbx/ubx)
    args=struct;     % Inicializa variável que vai armazenar a estrutura de argumentos do NLP
    args.lbx=[];       % Inicializa limites inferiores para as restrições dos estados do MPC (x0 até Hp+1, u0 até Hp)
    args.ubx=[];     % Inicializa limites superiores para as restrições dos estados do MPC

    % Observe que os estados X (medições) tem restrições dinâmicas, mas sabemos que lbx/ubx não podem conter fórmulas
    % Para limitar o espaço de busca do solver, estes limites podem ser definidos externamente.
    % As restrições, de fato, serão efetivamente tratadas em g

    % Para os estados atuais e para todo o horizonte Hp
    % Observar que X(1) são as medições atuais e impor restrições aqui poderia inicializar em unfeasible.
    % É necessário criar restrições apenas para cumprir a estrutura do modelo CaSAdi
    % Para os indices X(2) até X(Hp+1) que represetam os estados preditos nos horizontes 1:Hp,
    % as restriçoes serão tratadas de forma dinâmica em g
    for k=1:1+Hp % 1 é o instante atual e Hp é o horizonte de predicao
        args.lbx=[args.lbx, zeros(1,nx) ];      % Limites inferiores para as restrições de X (serão tratada em g)
        args.ubx=[args.ubx, inf(1,nx)];         % Limites superiores para as restrições de X (serão tratada em g)
    end

    % Para as ações de controle em todo o horizonte futuro, fazemos até Hp pois o U precisará ser calculado até
    % o horizonte final. Na ponderação da função custo, porém, entra apenas até Hc
    % Para as restrições da ação U na PMonAlvo, serão também consideradas as restrições dinâmicas (em função da frequência, atual e futura)
    % da PChegada (estados X). Mas isso será devidamente tratado nas restrições em [g]
    for k=1:Hp
        args.lbx=[args.lbx,    umin  ];             % Limites inferiores para a ação de controle U
        args.ubx=[args.ubx, umax ];             % Limites superiores para a ação de controle U
    end

    % Restrições para os limites na variação das ações de controle (especificidades serão tratadas em lbg/ubg)
    for k=1:Hp
        args.lbx=[args.lbx,  -dumax ];           % Limites inferiores para as variações na ação de controle U
        args.ubx=[args.ubx, dumax ];           % Limites superiores para as variações na ação de controle U
    end


    %% Montando as restrições de igualdade/desigualdade em g
    % Inicializa variável para guardar as fórmulas das restrições que vamos criar livremente
    % OBSERVAR QUE AS RESTRIÇÕES SÃO EMPILHADAS EM COLUNAS ENQUANTO SEUS
    % ARGUMENTOS (lbx/ubx, lbg/ubg) SÃO MONTADOS EM LINHA
    g=[];
    args.lbg=[];       % Inicializa limites inferiores para as restrições que vamos criar "livremente"
    args.ubg=[];     % Inicializa limites superiores para as restrições que vamos criar "livremente"

    % Multipleshooting = restrições de igualdade para seguir dinâmica do sistema
    % Define o X(1) como sendo as medições obtidas na entrada da planta
    g=[g; X(:,1) - Xk0];                               % Diferença entre os estados X e as medições do sistema
    args.lbg=[args.lbg,   zeros(1,nx)  ];     % Restrições de igualdade para impor a dinâmica do sistema
    args.ubg=[args.ubg, zeros(1,nx) ];

    % Limita a primeira ação de controle pelo máximo DeltaU, tendo como referência a ação de controle atualmente aplicada
    g=[g; U(:,1) - Uk0 - DU(:,1)];               % Garante que o primeiro DeltaU vai ter com referência a entrada atual
    args.lbg=[args.lbg,   zeros(1,nu)  ];     % Limite para a restrições de igualdade acima
    args.ubg=[args.ubg, zeros(1,nu) ];

    %% Restrições de igualdade para assegurar que os estados futuros vão seguir as predições
    for k=1:Hp
        % Estima passo futuro com base nos estados atuais e ações de controle atuais
        [x_predito, ModeloPreditor_novo_estado] = executa_predicao(U(:,k),X(:,k), ModeloPreditor_estado_atual, ModeloPreditor, EstimaVazao);
        ModeloPreditor_estado_atual=ModeloPreditor_novo_estado;     % Mantem a ESN com reservatório atualizado durante o loop de predições futuras

        % Restrições de igualdade para impor a dinâmica (técnica multipleshooting)
        g=[g;X(:,k+1)-x_predito];                          % Diferença entre os estado X futuro e o estado estimado pelo preditor
        args.lbg=[args.lbg,   zeros(1,nx)  ];          % Limite para a restrição de igualdade acima
        args.ubg=[args.ubg, zeros(1,nx) ];
    end

    %% Restrições de igualdade para definir DeltaU em função de U e tornar U como variável de decisão base
    for k=1:Hp-1
        % Cálculo da variação na ação de controle = DeltaU
        Soma=U(:,k+1)-U(:,k)-DU(:,k+1);
        g=[g;Soma];                                        % Restrição de igualdade = 0 assegura DeltaU como função de U
        args.lbg=[args.lbg,   zeros(1,nu) ];    % Limite mínimo para a restrição de igualdade associada ao DeltaU
        args.ubg=[args.ubg, zeros(1,nu)];    % Limite máximo para restrição de igualdade associada ao DeltaU

        % Aproveita o loop para criar restrição avaliando as variações acumuladas na frequencia
        BuffDeltaFreq=[ DU(1,k); BuffDeltaFreq(1:end-1)];    % Atualiza buffer com valor de DeltaFreq proposto
        Soma=sum(BuffDeltaFreq);
        % Avalia limites das variações
        g=[g; Soma];                      % Insere restrição de desigualdade para o somatório do BuffDeltaFreq
        args.lbg=[args.lbg,   -1 ];    % Limite mínimo para o somatório da variação
        args.ubg=[args.ubg,  1 ];  % Limite máximo para o somatório da variação
    end

    %% Restrições dinâmicas para os estados X e para as saidas Y controladas por setponit
    for k=1:Hp
        %             BUSCA RESTRIÇÕES DINÂMICAS PARA SEREM TRATADAS NO LOOP
        %             Lembrar que a linha 1 traz os limites máximos de todas as 11 variáveis do processo (10 + Vazão)
        %             Lembrar que a linha 2 traz os limites mínimos  de todas as 11 variáveis do processo (10 + Vazão)
        LimitesX= buscaLimites(U(1,k));  % Resgata limites (Max/Min) de alarmes para as variáveis do processo em função da frequência
        LimitesX=LimitesX';                                    % Coloca na forma de coluna

        LimitesX(:,1)=LimitesX(:,1)*(1-MargemPercentual/100);   % Implementa margem de folga em relação ao máximo
        LimitesX(:,2)=LimitesX(:,2)*(1+MargemPercentual/100);   % Implementa margem de folga em relação ao mínimo

        LimitesY=funcao_h(LimitesX);                                % Extrai limites correspondentes as saidas (variáveis controladas por setpoint)

        %             RESTRIÇÕES PARA AS VARIÁVEIS DO PROCESSO (ESTADOS X)
        %             Insere restrições para os valores máximos das variáveis (estados X) preditos
        LimMaxX=LimitesX(:,1)-X(:,k+1);    % Para não ser violado o limite, a diferença deve ser >= 0
        g=[g; LimMaxX];
        args.lbg=[args.lbg,   zeros(1,nx) ];   % Limite mínimo para restrição de desigualdade
        args.ubg=[args.ubg,      inf(1,nx) ];   % Limite máximo para restrição de desigualdade

        %            Insere restrições para os valores mínimos das variáveis (estados X) preditos
        LimMinX=X(:,k+1)-LimitesX(:,2);      % Para não ser violado o limite, a diferença deve ser >= 0
        g=[g; LimMinX];
        args.lbg=[args.lbg,   zeros(1,nx) ];    % Limite mínimo para restrição de desigualdade
        args.ubg=[args.ubg,      inf(1,nx) ];    % Limite máximo para restrição de desigualdade

        %            RESTRIÇÕES PARA AS VARIÁVEIS DE SAIDA (CONTROLADAS POR SETPOINT)
        %            Insere restrições para os valores máximos das saidas controladas por setpoint que são preditas
        y_saida= funcao_h(X(:,k+1));                       % Saidas preditas (variáveis controladas por setpoint)

        LimMaxY=LimitesY(:,1)-y_saida;    % Para não ser violado o limite, a diferença deve ser >= 0
        g=[g; LimMaxY];
        args.lbg=[args.lbg,    zeros(1,ny) ];   % Limite mínimo para restrição de desigualdade
        args.ubg=[args.ubg,       inf(1,ny) ];   % Limite máximo para restrição de desigualdade

        %            Insere restrições para os valores mínimos saidas controladas por setpoint que são preditas
        LimMinY=y_saida-LimitesY(:,2);      % Para não ser violado o limite, a diferença deve ser >= 0
        g=[g; LimMinY];
        args.lbg=[args.lbg,  zeros(1,ny) ];    % Limite mínimo para restrição de desigualdade
        args.ubg=[args.ubg,     inf(1,ny) ];    % Limite máximo para restrição de desigualdade

        %            Para as restrições da ação U na PMonAlvo, serão também consideradas as restrições dinâmicas da PChegada (estados X)
        %            Lembrar que a ação de controle é na Freq e PMonAlvo, portanto, umin(2) e umax(2)
        %            associam limites pré-estabelecidos para a PChegada
        %            Já em relação aos estados X, a PChegada é a variável na linha 2

        %            LIMITES MINIMOS PMonAlvo
        ValMin=max(LimitesX(2,2),umin(2));   % Assume o valor mais restritivo
        DiferencaMin=U(2,k)-ValMin;              % Ação precisa ser maior do que o minimo
        g=[g; DiferencaMin];                              % Insere restrição de desigualdade, a qual precisa ser  >=0
        args.lbg=[args.lbg,    0 ];                        % Limite mínimo
        args.ubg=[args.ubg, inf];                        % Limite máximo

        %            LIMITES MÁXIMOS PMonAlvo
        ValMax=min(LimitesX(2,1),umax(2));   % Assume o valor mais restritivo
        DiferencaMax=ValMax-U(2,k);            % Ação precisa ser menor que o máximo
        g=[g; DiferencaMax];                              % Insere restrição de desigualdade, a qual precisa ser  >=0
        args.lbg=[args.lbg,    0 ];                        % Limite mínimo
        args.ubg=[args.ubg, inf];                        % Limite máximo
    end

    % Depois do instante Hc, seguir a teoria e manter a mesma ação de controle futura.
    % No caso específico, julgamos ser importante termos a ação de  controle calculada até o último horizonte,
    % uma vez que cada uma delas implica em diferentes restrições dinâmicas futuras. Assim, na sintonia do
    % controlador, usualmente faremos Hc=Hp. Mas para atender a teoria, podemos implementar as restrições de igualdade
    % para os casos em que o Hc seja menor
    for k=Hc+1:Hp
        g=[g;U(:,k)-U(:,k-1)];                         % Restrição de igualdade, assegura ação U igual a anterior
        args.lbg=[args.lbg,   zeros(1,nu) ];    % Limite mínimo para a restrição de igualdade
        args.ubg=[args.ubg, zeros(1,nu)];    % Limite máximo para restrição de igualdade
    end

    %% Preparando o custo da função objetivo
    % Lembrar que X(:,1) são as medidas atuais. Da coluna 2 em diante teremos os estados futuros estimados de 1 até Hp
    fob=0;                                                           % Inicializa custo da função objetivo
    fob=fob+ErroX'*Qx*ErroX;                         % Incrementa custo do erro de predição dos estados X

    for k=1:Hp                                                    % Para todo o horizonte de predição
        % Incrementa custo com a diferença entre as saidas estimadas e o setpoint desejado
        % Observar que o ErroY entra para zerar offset provocado pelo erro do estimador
        y_saida= funcao_h(X(:,k+1));                             % Saida estimada (variáveis controladas por setpoint - retorna coluna) 
        fob=fob+(y_saida-Ysp+ErroY)'*Qy*(y_saida-Ysp+ErroY);
    end

    for k=1:Hc             % Para o horizonte de controle Hc
        % Incrementa custo com a diferença entre a ação de controle e o AlvoEng
        % Incrementa custo da função objetivo com o valor de DeltaU
        S=(U(:,k)-AlvoEng)'*Qu*(U(:,k)-AlvoEng) + DU(:,k)'*R*DU(:,k);
        fob=fob+S;
    end


    %% ========================Configuração do otimizador====================================
    % Monta as variáveis de decisão em um vetor coluna - esta dimensão é fundamental para entender as contas e indexações
    % Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo HP  +        U             +     DeltaU      ]
    %                             Dimensão = [                  nx*(1+Hp)                                                    nu*Hp                 nu*Hp     ]
    opt_variables=[X(:); U(:);  DU(:)];

    nlp = struct('f',fob,'x',opt_variables,'g', g, 'p', P); % Define a estrutura para problema de otimização não linear (NLP, Nonlinear Programming)

    %Configuração específica do IPOPT
    options=struct;
    % Parâmetros para análise/depuração
    options.print_time= 0;                           %
    options.ipopt.print_level=0;                   % [ 0 a 12] = (funciona 3 a 12) Detalhe do nivel de informação para mostrar na tela durante a execução do solver
    options.verbose = 0;
    options.ipopt.timing_statistics ='yes';       % Permite visualizar dados estatísticos dos processo do solver
%     options.ipopt.print_options_documentation = 'yes';
%     options.ipopt.print_user_options = 'yes';

    % Parâmetros que podem ser definidos externamente - padronizamos 9s no caso epecífico Petrobras
    options.ipopt.max_wall_time=WallTime;  % Tempo (em segundos) máximo para o solver encontrar solução

% Parâmetros alterados em relação ao padrão do Solver
    options.ipopt.bound_relax_factor=0;    % Tolerância absoluta para as restrições definidas pelo usuário (default=1e-8)
    options.ipopt.hessian_approximation = 'limited-memory';    % This determines which kind of information for the Hessian of the Lagrangian function is used by the algorithm. The default value for this string option is "exact".
   options.ipopt.max_iter=1e4;                     % Especifica o número máximo de iterações que o solver deve executar antes de parar.
    
% Parâmetros avaliados mas que concluimos por manter o padrão do Solver
% options.ipopt.mu_init=1e-6;   ou  options.ipopt.mu_init=1e-1;
% options.ipopt.hessian_constant = 'yes';   % Flag indicating if we need to ask for Hessian only once
% options_solver.ipopt.warm_start_init_point=  'yes';
% options_solver.ipopt.dependency_detector=  'MUMPS';
% options_solver.ipopt.linear_solver=  'MUMPS';
% options_solver.ipopt.dependency_detection_with_rhs= 'yes';
% options_solver.ipopt.fixed_variable_treatment=  'make_parameter';
% options_solver.ipopt.honor_original_bounds=  'yes';

% Condições testadas mas que se mostraram inviáveis por UNFEASIBLE
% grad_f_constant=  'yes';
% warm_start_same_structure=  'yes';    
% nlp_scaling_method='equilibration-based';

    solver = nlpsol('solver','ipopt', nlp, options); % Define o Interior Point OPTimizer (ipopt) para resolver o problema de otimização não linear (nlp)
    t_inicializacao = toc(tInicializa);           % Tempo gasto para a inicialização do Solver
    disp(strcat("Tempo para inicialização = ",num2str(t_inicializacao)))
end


%% Função para extrair partes que compõe os parâmetros do Solver
function   [ Xk0,Uk0,AlvoEng,Ysp,ErroX, ErroY, BuffDeltaFreq, Reservatorio_ESN]=ExtraiParametros(P,Indice)
    Ate=[Indice(1)   sum(Indice(1:2))  sum(Indice(1:3))  sum(Indice(1:4))  sum(Indice(1:5))  sum(Indice(1:6))  sum(Indice(1:7))  sum(Indice(1:8)) ];
    De=[1  Ate(1)+1   Ate(2)+1   Ate(3)+1   Ate(4)+1   Ate(5)+1   Ate(6)+1 Ate(7)+1];
    Xk0=P(De(1):Ate(1));
    Uk0=P(De(2):Ate(2));
    AlvoEng=P(De(3): Ate(3));
    Ysp=P(De(4):Ate(4));
    ErroX=P(De(5):Ate(5));
    ErroY=P(De(6):Ate(6));
    BuffDeltaFreq=P(De(7):Ate(7));
    Reservatorio_ESN=P(De(8):Ate(8));
end






