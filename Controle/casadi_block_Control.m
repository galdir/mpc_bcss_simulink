%%  IMPLEMENTAÇÃO MPC USANDO A BIBOIOTECA CASADI
classdef casadi_block_Control < matlab.System & matlab.system.mixin.Propagates
    properties (DiscreteState)
    end
    properties (Access = private)                       % Criação das variáveis que vão compor o OBJETO
        casadi_solver                                            % Criação do solver Casadi
        x0                                                                % Criação da variável para guardar os estados para operação do MPC
        lbx                                                                % Lower Bounds para os Estados do MPC
        ubx                                                               % Upper Bounds para os Estados do MPC
        lbg                                                                % Lower Bounds para as restrições [g] que forem criadas
        ubg                                                               % Upper Bounds para as restrições [g] que forem criadas
        
        contador                                                     % Criação da variável para guardar o contador de loops - define momentos de atuação do MPC
        ModeloPreditor                                           % Criação da variável para guardar modelo de preditor do processo e que será utilizada pelo solver para a predição
        MatrizLimitesDinamicos                            % Matriz correspondente a Tabela com resultados dos limites Max/Min de proteção dinâmica para todas as frequências
       
    end
%%======================================================
    methods (Access = protected)
        %% ============   Ajuste das entradas
        function num = getNumInputsImpl(~)        % Nùmero de pinos de entradas do bloco CASADI (ver no Simulink)
            num = 13;                                               % São 13 pinos
            %    13 = 10 variáveis do processo + VazãoEstimada + 2 manipuladas (leitura da Freq e da PMon aplicadas ao processo)
            %     1  = clock = tempo de simulação dado pelo Simulink
            %     2  = Alvos = Valores alvo para as manipuladas (Freq e PMon)
            %     2  = dumax = Delta U máximo que pode ser praticado em cada uma das manipuladas  
            %     2  = dumin = Delta U mínmo que pode ser praticado em cada uma das manipuladas  
            %     1  = Hp = Horizonte de predição do controlador MPC
            %     1  = Hc = Horizonte de controle MPC
            %     2  = Qy = Matriz de supressão das variáveis controladas
            %     2  = R = Matriz de supressão das variáveis manipuladas
            %     2 = Qu = Matriz de supressão dos alvos definidos pelo usuário
            %     1 = Amostra MPC = Taxa de amstragem proporcional do MPC em relação aos dados de amostragem do processo
            %     1 = Umax (Freq e PmonAlvo Máximos)
            %     1 = Umin (Freq e PmonAlvo Mínimos)

        end
        %===============        
        function sz1 = getInputSizeImpl(~)            % Organização das entradas do bloco Casadi
            sz1=13; 
        end
        %===============        
        function dt1 = getInputDataTypeImpl(~)   % Tipo das variáveis de entrada - usamos todas "double"
            dt1 = 'double';
        end   
        %===============     
        function cp1 = isInputComplexImpl(~)
            cp1 = false;
        end
        %===============
        function fz1 = isInputFixedSizeImpl(~)
            fz1 = true;
        end
        %===============        
        %% ============  Ajuste das saidas
        function num = getNumOutputsImpl(~)     % Numero de pinos na saida do bloco Casadi
            num=1;
            % num = 6;     %Poderia ser NUM=6 e colocarmos a implementação na forma   [1;1;2;2;2;11]
        end
        %===============        
        function sz1 = getOutputSizeImpl(~)         % Organização da saida do bloco Casadi
           sz1 = 1+1+2+2+2+11;                            % Dimensão será de 19
           % sz1 = [1;1;2;2;2;11];                            % Dimensão total será de 19, se quiser colocar na forma de 6 saídas
           % 1 = Informação de realizável (feasible) vindo do otimizador. (1/0) Indica se deu tempo de achar solução ótima
           % 1 = Tempo gasto pelo otimizador 
           % 2 = Valores calculados para as manipuladas Freq. e PMonAlvo
           % 2 = Variações ótimas calculadas para as manipuladas (Delta U de Freq e PMonAlvo). Será útil para visualizar a velocidade das alterações
           % 2 = Álvos ótimos calculados para a PSuc e PChegada
           % 11 = Pedições do Processo (só para visualização), incluindo a vazão
        end
        %===============        
        function dt1 = getOutputDataTypeImpl(~)   % Tipo das variáveis de saída - usamos todas "double"
            dt1 = 'double';
        end
        %===============        
        function cp1 = isOutputComplexImpl(~)
            cp1 = false;
        end
        %===============        
        function fz1 = isOutputFixedSizeImpl(~)
            fz1 = true;
        end
        %===============
        %%  =========== Outras funções do bloco Casadi
%         function resetImpl(obj)
%         end
        %===============        
 %% ================     Inicialização dos parâmetros - só passa aqui uma única vez (equivale a Flag=0 na SFunction)
       function setupImpl(obj,~)
            import casadi.*                                    % Importação da biblioteca Casadi (tem de estar no path)
            
            %% Carrega tabelas Petrobras
            % Tabelas para cálculos das proteções dinâmicas
            % Transforma tabelas em matrizes para melhorar desempenho computacional e permitir o uso de algébra simbólica (CaSAdi)
            TabelaLimitesDinamicos =evalin('base', 'TabelaLimitesDinamicos');       % Tabela com limites Max/Min de todas as variáveis em todas as frequências (com resolução de 0,1)
            MatrizLimitesDinamicos = table2array(TabelaLimitesDinamicos);
            obj.MatrizLimitesDinamicos = table2array(TabelaLimitesDinamicos);

            % Tabela do Simulador para cálculos da vazão por interpolação
            TabelaSimulador=evalin('base', 'TabSimulador');                               % Tabela do Simulador para cálculos da vazão
            MatrizSimuladorVazao = table2array(TabelaSimulador(:,1:3));        % Especificamente as colunas Freq, PChegada e Vazao para diminuir o esforço computacional
            
            %% Carrega parâmetros definidos pelo usuário
            % Do controlador
            Hp =  evalin('base','Hp');                     % Horizonte de predição
            Hc =  evalin('base','Hc');                     % Horizonte de controle
            Qy=  evalin('base','Qy');                      % Peso das saidas controladas por setpoint
            Qx= evalin('base','Qx');                        % Peso para os erros de estimação das variáveis do processo
            Qu = evalin('base','Qu');                      % Peso das ações de controle nas entradas (Alvos Desejados = Freq. e PMonAlvo)
            R = evalin('base','R');                          % Peso das ações de controle nas entradas (Alvos Desejados = Freq. e PMonAlvo)
          
           % R =  diag(repmat(R,1,Hc));                  % Expande R em todo o Hc
         
            % Dos dados de operação
            umax =  evalin('base','umax');             % Valor máximo para cada uma das variáveis de entrada (variáveis manipuladas)
            umin =  evalin('base','umin');               % Valor mínimo para cada uma das variáveis de entrada (variáveis manipuladas)
            dumax =  evalin('base','dumax');          % Variação máxima do delta U nas entradas (variáveis manipuladas)
            dumin =  evalin('base','dumin');            % Variação mínima do delta U nas entradas (variáveis manipuladas)
            
            %% Carrega condições inciais conhecidas para inicialização dos estados X0 do MPC
            XIni=evalin('base','XIni');                       % Condições iniciais das variáveis (estados) do processo
            UIni=evalin('base','UIni');                       % Condições iniciais das entradas (variáveis manipuladas) do processo
            nu=height(UIni);
            nx=height(XIni);
                      
            %% Carrega matriz que faz o papel de Y=h(x), onde a saída Y(k) é função dos estados X(k)
            matriz_h=evalin('base','matriz_h');        % matriz para calcular as saidas na forma y=h(x)
            EstadosMedidos=MX.sym('HdeX',nx,1); 
            h=Function('h',{EstadosMedidos},{matriz_h*EstadosMedidos}); 

            %% Inicializa estados para atuação do MPC
            YIni=matriz_h*XIni;                               % Extrai de X os valores iniciais das saidas, antes de expaldir os estados no Hp   
            ny=height(YIni);
            
            % Expande as matrizes nos respectivos horizontes (padrão Casadi para o MPC)
            XIni= repmat(XIni,1+Hp,1);                  % Expande XIni considerando estado atual + todo o horizonte Hp 
            UIni= repmat(UIni,Hc,1);                      % Expande UIni  para todo o horizonte Hc
%             R =  diag(repmat(R,1,Hc));   % Já veio expandido pois o repmat não funcionou aqui

            x0=[XIni;UIni;YIni];    % Monta vetor de estados iniciais X0 para operação do MPC (é para eles que teremos de criar os limites lbx e ubx)

            obj.x0 = x0;                                             % Condição inicial para operação do MPC  
%            obj.contador = -1;                                   % Contador para indicar passos/momentos de atuação do controle MPC (-1 para contagem de tempo sincronizar com passos de 30)
            obj.contador = 0;                                   % Contador para indicar passos/momentos de atuação do controle MPC (-1 para contagem de tempo sincronizar com passos de 30)
          
            %% Carrega modelo preditor e esquenta a ESN
            ModeloPreditor = evalin('base', 'ModeloPreditor');   % Modelos do preditor que será usada pelo MPC
            TipoPreditor=ModeloPreditor.data.tipo;                    % Verifica tipo 1=ESN, 2 = LSTM
            EntradasESN_Normalizadas = normaliza_entradas([UIni;XIni]);   % Normaliza entradas provenientes do processo (observar que a função nada faz com a vazão)
            ModeloPreditor.data.a0 = esquenta_ESN(ModeloPreditor.data,EntradasESN_Normalizadas,1000); % Atualiza várias vezes o estado do reservátório para esquentar ESN
            obj.ModeloPreditor = ModeloPreditor;   % ESN precisará ter seus estados internos atualizados a cada amostragem

             %% Funções simbólicas que serão necessárias ao Solver
            f_Interpola_casadi_vazao_sym = evalin('base', 'f_Interpola_casadi_vazao_sym'); 
            f_buscaLimites_sym=evalin('base', 'f_buscaLimites_sym'); 
            
            %% Montar restrições 
            
           
            %% Inicialização do SOLVER (antigo InicializaSolver)
            % Cria o Solver em formato simbólico em formato para o CaSAdi
            tInicializa=tic;                                        % Marcador para o tempo gasto no Solver
            
            %% ========================Define parâmetros simbolicos (Casadi) gerais para o problema de otimização=====================================
            X =     MX.sym('X',nx,Hp+1);                           % Predição dos estados sobre o horizonte Hp  
            du =    MX.sym('du',Hc*nu,1);                          % Incrementos do controle sobre o horizonte Hc (Variável de decisão)
            Du =    [du;zeros(nu*(Hp-Hc),1)];                    % Sequencia de ação de controle
            ysp =   MX.sym('ysp',ny,1);                              % Set-point otimo calculado pelo otimizador (Variável de decisão)

            %% ========================Escolha o modeloPreditor que será usando no MPC (1=ESN, 2=LSTM)  =====================================
            %Parâmetros simbólicos específicos da ESN
            nx_ESN =      length(ModeloPreditor.data.Wir);   % Resgata o tamanho do reservatório da ESN utilizada como modelo preditor

            % P = quantidade de parâmetros para o Solver. Os P parâmetros são:
            % - DadosProcesso (dimensão=nx)
            % - uk(entradas) (dimensão=nu)
            % - Erro, sendo a diferença entre a medição do processo e a última predição das variáveis controladas (dimensão=ny)
            % - Alvo dado pelo RTO (dimensão=nu)
            % - Dados do reservatório da ESN utilizada pelo controlador (dimensão=nx_ESN)
            P =           MX.sym('P',nx+nu+ny+nu+nx_ESN);    % qtd de parâmetros para o Solver
            uk_1 =        P(nx+1:nx+nu);                                    % define variável simbólica das entradas (Freq. PmonAlvo)
            erro =        P(nx+nu+1:nx+nu+ny);                         % define variável simbólica para erro (DadosProcesso-PrediçãoMPC) ->(Psuc e Pcheg)
            uRTO =        P(nx+nu+ny+1:nx+nu+ny+nu);         % define variável simbólica para Alvo (Freq. e PmonAlvo)
            ESNdataa0 =   P(nx+nu+ny+nu+1:end);              % define variável simbólica do reservatório da ESN
            g=[X(:,1)-P(1:nx)];                                                  % define variavel que vai empilhar as restrições durante o Hp

            % Define a função objetivo (fob) de forma recursiva ao longo de Hp passos, utilizando o modelo preditor para otimizar as variáveis de controle, considerando as restrições do processo.
            for k=1:Hp
                uk_1 = uk_1 + Du((k-1)*nu+1:k*nu);           % define variável simbólica para soma dos incrementos de controle
                LL= f_buscaLimites_sym(uk_1(1));             % rascunho !!!

                ym = h(X(:,k+1));                                           % define variável simbólica que será controlada utilizando a função de saída (h) definida anteriomente 
                fob=(ym-ysp+erro)'*Qy*(ym-ysp+erro)+du'*R*du+(uk_1-uRTO)'*Qu*(uk_1-uRTO);            % define a função objetivo proposta

                EstadosMedidos=P(1:nx);
%                 EstadosMedidos=X(:,1);   % Why not X???
%                 EntradaModeloPreditor = [uk_1;EstadosMedidos];                                         % Define uma matriz para armazenar as variáveis de entrada no modeloPreditor

                [y_esn_pred, ESNdataa0] = executa_predicao(uk_1,EstadosMedidos , ESNdataa0, ModeloPreditor, f_Interpola_casadi_vazao_sym);

                g=[g;X(:,k+1)-y_esn_pred];   % Define variável simbólica para compor as restrições nos LimitesInferior e LimiteSuperior(lbg<g(x)<ubg)                                                                     
            end
            %% ========================Define as matrizes auxiliares (Mtil, Itil) para o incremento de controle (ação de controle) ao longo de Hc passos, no conjunto de restrições (g)====================================
            Mtil=[];                         
            Itil=[];
            auxM=zeros(nu,Hc*nu);
            for in=1:Hc
                auxM=[eye(nu) auxM(:,1:(Hc-1)*nu)];
                Mtil=[Mtil;auxM];
                Itil=[Itil;eye(nu)];
            end
            % Conclui a inclusão das restrições nos estados e nas entradas.
            g = [g;Mtil*du+Itil*P(nx+1:nx+nu)]; 

            %% ========================Configuração do otimizador====================================
            opt_variable=[X(:);du;ysp];                   %variáveis calculadas pelo Solver(predição;incrementos de controle;set-point*) 
            nlp = struct('f',fob,'x',opt_variable,'g', g, 'p', P); %define a estrutura para problema de otimização não linear (NLP, Nonlinear Programming), sendo: 1-fob, definida acima; 2-opt_variable: variáveis de decisão; 3-g, as restrições do processo e 4-P, parâmetros de entrada para Solver   

            %Configuração específica do otimizador
            options=struct;
            options.print_time=0;                         % Habilita tempo total de execução do solver deve ser impresso ou não.
            options.ipopt.print_level=1;                  % Nível de detalhamento das mensagens de saída do IPOPT. Valores mais baixos resultam em menos mensagens (0 significa sem mensagens).
            options.ipopt.max_iter=100;                   % Especifica o número máximo de iterações que o solver deve executar antes de parar.
            options.ipopt.acceptable_tol=1e-4;            % Define a tolerância de convergência do solver. Um valor menor indica uma solução mais precisa.
            options.ipopt.acceptable_obj_change_tol=1e-4; % Define uma tolerância aceitável para uma solução "boa o suficiente", útil para problemas onde a solução perfeita pode ser muito difícil de alcançar.
            obj.casadi_solver = nlpsol('MontagemSolver','ipopt', nlp,options); % Define o Interior Point OPTimizer (ipopt) para resolver o problema de otimização não linear (nlp)            
  
            t_inicializacao = toc(tInicializa);           % Tempo gasto para a inicialização do Solver
%             disp(strcat("Tempo para inicialização do Solver = ",num2str(t_inicializacao)))

% IMPORTANTE:
% Como está:
% Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo HP  +  Ações de controle em todo Hc + Saidas]
%                             Dimensão = [        nx                      nx*Hp                                                 nu*Hc                                        ny   ]

% Possivelmente as saidas ny foram colocadas aqui para que pudessem ser geradas as restrições.
% Se assim o for, não precisaria pois as restrições da saida podem ser tratadas em g
% Talvez devesse ser algo como:
% Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo HP  +  Ações de controle em todo Hc ]
%                             Dimensão = [        nx                      nx*Hp                                                 nu*Hc ]

            
        end
%% ================  Contas de atualização -  Equivale a Flag=2 na SFunction)
        function Saida = stepImpl(obj,DadosProcesso,t,Alvos,dumax,dumin,Hp,Hc,Qy,R,Qu,PassoMPC,umax,umin)
%         disp(strcat("Simulação MPC em ",num2str(t)," s      Passo MPC = ",num2str(PassoMPC)))
             EstruturaSolver = obj.ModeloPreditor.data.tipo;         % Extrai tipo do preditor (1=ESN; 2 = LSTM)
            %% ===================== Configuração inicial do MPC  ========================================
            [ ny, nu, nx] = DimensoesVariaveisProcesso(Qy,Qu,DadosProcesso);  % Lembrar que DadosProcesso estão trazendo Freq e PMon aplicadas ao processo
            UProcesso=DadosProcesso(nx+1:nx+nu);  % Extrai ações (Frequencia e PMonAlvo) aplicadas ao processo
            DadosProcesso=DadosProcesso(1:nx);      % Extrai apenas as nx variáveis com dados do processo
            
            % Prepara dados para atualizar a ESN  (AVALIAR SE ESTE É O LOCAL CORRETO )
            EntradasESN_Normalizadas = normaliza_entradas([UProcesso;DadosProcesso]);   % Normaliza entradas provenientes do processo (observar que a função nada faz com a vazão)
            
            %% ===================== Dados do processo e da predição ========================================
             PredicaoHorizonteHp=obj.x0(nx*(Hp)+1:nx*(Hp)+nx);                     % Busca variáveis preditas no horizonte Hp (futuro mais distante possível) 
            % Lembrar que as "nu" variáveis controladas (PSuc e PChegada) são as 2 primeiras do vetor de variáveis do processo
            erro =    DadosProcesso(1:nu)-PredicaoHorizonteHp(1:nu);            % Diferença entre a medição do processo e a última predição das variáveis controladas (dimensão=nu)
            ysp =     obj.x0(nx*(Hp+1)+Hc*nu+1:end);                                           % Resgata solução ótima (set-points) do MPC antes dada pelo otimizador
                        
            %% ===================== Empilha restrições para as variáveis do processo  =============================
            % Colocando na mesma organização do bloco de cálculo das restrições que foram recebidas nas entradas
        
             % Calculando as restrições com base na tabela já prédefinida
             % Precisaremos levar isso para os cálculos das restrições futuras
            Freq=round(UProcesso(1),1);   % Arredonda em uma casa decimal - isso é feito considerando delta Freq Minimo =0.1 (Poderiamos interpolar, mas não parece mercer)
            Restricoes = obj.MatrizLimitesDinamicos(obj.MatrizLimitesDinamicos(:,1)==Freq, :);
            % Extrai limites para as variáveis em função da frequencia de operação
            RestricoesMax = Restricoes(1, 2:end)';    
            RestricoesMin = Restricoes(2, 2:end)';

            % Necessário agora concatenar limites das variáveis de processo (11=10+vazão) + restrições de manipulação (DeltaU) em
            % todo o horizonte hp; e das controladas (PSuc e PChegada), que são as mesmas variáveis 1 e 2 do processo
            
            % Usando a faixa da PChegada como sendo os limites do mapa
            LimitesMax= [repmat(RestricoesMax,(Hp+1),1);repmat(dumax,Hc,1); [RestricoesMax(1) ;RestricoesMax(2)]];
            LimitesMin = [repmat(RestricoesMin,(Hp+1),1);repmat(-dumax,Hc,1); [RestricoesMin(1) ;RestricoesMin(2)]];

%             LimitesMax= [repmat(RestricoesMax,(Hp+1),1);repmat(dumax,Hc,1); [Psuc(1) ;30]];
%             LimitesMin = [repmat(RestricoesMin,(Hp+1),1);repmat(-dumax,Hc,1); [Psuc(2) ;30]];
%             % Usando a faixa da PChegada como sendo os limites máximos definidos pelo usuário
%             LimitesMax= [repmat(RestricoesMax,(Hp+1),1);repmat(dumax,Hc,1); [Psuc(1) ;umax(2) ]];
%             LimitesMin = [repmat(RestricoesMin,(Hp+1),1);repmat(-dumax,Hc,1); [Psuc(2) ;umin(2) ]];
 
%% ================ Empilha restrições para as variáveis manipuladas ==================================
            ManipuladasLowLimit = [repmat(zeros,nx*(Hp+1),1);repmat(umin,Hc,1)];   %Atribui em obj. as restrições para solver (limites inferiores)
            ManipuladasHighLimit = [repmat(zeros,nx*(Hp+1),1);repmat(umax,Hc,1)]; %Atribui em obj. as restrições para solver (limites superiores)
            

            %% ===================== %Parâmetros e atuação do solver ========================================
            obj.contador = obj.contador+1;     % Contador ajudará a saber se é para o Solver do Otimizador/Controlador atuar
            Tsolver=0;                 % Inicializa contados para calcular o tempo gasto com o Solver, quando for o caso !!
            DeltaU=[0;0];             % Assume variação nula. Será alterada ao passao pelo solver
            Feasible=0.5;            % Assumir padrão para indicar que não passou pelo Solver

           if EstruturaSolver==1
                par_solver =    [DadosProcesso;UProcesso;erro;Alvos;obj.ModeloPreditor.data.a0];    %DadosProcesso; UProcesso = última ação de controle; Erro entre medição e predição; Alvos ENG; Estado do reservatório da ESN
            end
            
            %% ========= Conta do Solver para Galdir
            if (obj.contador==PassoMPC)      % Solver só entra no passo definido pelos parâmetros de Passo do MPC
                tStart=tic;                                                                     % Dispara contagem para medir o tempo do solver
                solver_MPC=obj.casadi_solver('x0',obj.x0,'lbx', LimitesMin,'ubx', LimitesMax,'lbg', ManipuladasLowLimit, 'ubg', ManipuladasHighLimit, 'p', par_solver);
                Feasible=obj.casadi_solver.stats.success;             % Atualizar status do Feasible
                % Usa resposta do solver para atualizar variáveis
                obj.x0 = full(solver_MPC.x);                                       % Atualiza objeto com solução ótima no instante k (PredicaoHorizonteHp; Deltau_k, Ysp)
                ResultadoMPC=obj.x0;                                              % Vetor com resultados proveniente da otimização feita pelo MPC
                DeltaU=ResultadoMPC(nx*(Hp+1)+1:nx*(Hp+1)+nu);  % Extrai a ação de controle ótima (apenas para o instante k+1)
                for i=1:nu                                                                    % Para cada uma das variáveis manipuladas
                    DeltaU(i)=AvaliaDeltaU(DeltaU(i),dumin(i));        % Avalia se pode dar o passo e observa limites minimos (o máximo não precisa testar aqui pois o limite da ação vai ser tratado)
                end
                % Atualiza ação de controle com DeltaU resultante
                UProcesso = UProcesso + DeltaU;          % Aplica na saida a solução ótima (Uk + Delta Uk calculado pelo solver)
                
                % Extrai os novos set-points ótimos calculados
                ysp=ResultadoMPC(end-ny+1:end);                         % Extrai novos set-point ótimos (sempre no final do vetor que montamos para o MPC)
                
                obj.contador =0;                                                          % Reinicia contador para o passo do MPC
                Tsolver = toc(tStart);                                                   % Calcula tempo de atuação do solver
            end
            % Passando ou não pelo Solver, atualiza a ação de saida do controlador
            UProcesso=min(UProcesso,umax);     % Restringe as manipuladas (Freq e PMonAlvo) nos limites máximos pré-definidos
            UProcesso=max(UProcesso,umin);     % Restringe as manipuladas (Freq e PMonAlvo) nos limites mínimos pré-definidos
            
            %% ============    Mantém ESN atualizada   =======================
            % Atualiza o estado do reservátório (1 passo) para proxima predição
            if EstruturaSolver==1
                QtePassosESN=1;       % Quantidade de passos para manter a rede esquentada. No caso, apenas 1 passo
                 obj.ModeloPreditor.data.a0 = esquenta_ESN(obj.ModeloPreditor.data,EntradasESN_Normalizadas,QtePassosESN);
            end
            
            %% ============    Saida com todas as variáveis calculadas pelo otimizador   =========
            Saida = [Feasible;Tsolver;UProcesso;DeltaU;ysp;PredicaoHorizonteHp];      
           % Feasible (Dim=1), informação de realizável (feasible) vindo do otimizador. (1/0) Indica se deu tempo de achar solução ótima
           % Tsolver (Dim=1), tempo gasto pelo otimizador 
           % UProcesso (Dim=2), valores calculados para as manipuladas Freq. e PMonAlvo
           % DeltaU (Dim=2), variações (Delta U) ótimas calculadas para as manipuladas (Delta U de Freq e PMonAlvo). Será útil para visualizar a velocidade das alterações
           % ysp (Dim=2), alvos ótimos calculados para a PSuc e PChegada
           % PredicaoHorizonteHp (Dim=10+1), são as Pedições do Processo feitas pelo MPC
           
        end
    end
end
%===============================================================================
% ==================  FIM DO PROGRAMA PRINCIPAL  ================================
%===============================================================================

%% ==================        Outras funções adicionais   ================================
%% ==============================================================================
%% Função para ter as dimensões das variáveis em função dos parâmetros indicados pelo usuário na inicialização
function [ny, nu, nx] = DimensoesVariaveisProcesso(Qy,Qu,DadosProcesso);
    ny=length(Qy);         % Associa número de variáveis controladas (no caso, 2: PSuc e PChegada)
    nu=length(Qu);         % Associa número de variáveis manipuladas (no caso, 2: Freq e PMonAlvo)
    T=length(DadosProcesso);    % Tamanho do vetor de dados do processo, mas precisa descontar as manipuladas que vieram junto
    nx = T-nu;                 % Associa número de variáveis coletadas do processo (no caso, 11 = 13 menos as 2 manipuladas)
end
%% ==============================================================================
%% Função para ajustar o DeltaU mínimo - ao menos enquanto não for tratada na formulação do solver
function RetornoDeltaU=AvaliaDeltaU(DeltaU,dumin);
    RetornoDeltaU=DeltaU;                                     % Retorna valor valor padrão calculado, sem correção pelo mínimo
    %     RetornoDeltaU=max([DeltaU,dumin]);       % Retorna valor limitado pelo deltaU minimo. Mas se deixar só assim, a ação de controle vai oscilar
    if dumin>0                                                             % Se foi especificado um deltaU minimo > 0  (se for=0, passa qq valor)
       CasasDecimais=countDecimals(dumin);        % Quantidade de casas decimais definidas para o limite minimo
       RetornoDeltaU=round(DeltaU,CasasDecimais);   % Retorna o DeltaU arredondado pelo numero de casas decimais com a resolução definida pelo valor minimo
    end
end
%% ==============================================================================
%% Função para contar numero de casas decimais
function num_decimals = countDecimals(number)
    str = num2str(number);                                  % Converte o número para string
    decimalPos = find(str == '.', 1);                      % Encontra a posição do ponto decimal
    if isempty(decimalPos)                                  % Se não houver ponto decimal, o número é inteiro
        num_decimals = 0;
    else                                                                % Caso contrário, conta os dígitos após o ponto decimal
        num_decimals = length(str) - decimalPos;
    end
end
%% ==============================================================================
%% Função para execução a função de predição x(k+1)=f(xk,uk)
function [predicoes, novo_a0] = executa_predicao(EntradaU,EstadosX, ESNdataa0, modelo_ESN, f_matrizVazao_sym)
    %%
    % EntradaU são as entradas do processo no instante atual : frequencia_aplicada, pressao_montante_alvo_aplicada
    % EstadosX são as 11 variáveis do processo no instante atual:
    %           pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
    %           temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, 
    %           temperatura_succao_BCSS, vibracao_BCSS,  temperatura_chegada, VazãoOleo
    %   
    % modelo_ESN precisar ter: .data.Wrr, .data.Wir, .data.Wbr
    %
    % ESNdataa0 é o estado atual do reservatorio
    %
    % matrizVazao é a matriz com vazoes estimadas em função da frequencia e pressao de chegada
    %
    % saidas é um vetor coluna com as predições dos novos estados referentes as 11 variáveis do processo:

    % Executa predição da ESN
    [predicoes, novo_a0] = executa_predicao_ESN(EntradaU,EstadosX, ESNdataa0, modelo_ESN);
    % Executa predição da vazão (não é feita pela ESN)
    Freq=EntradaU(1);
    PChegada=predicoes(2)*1.019716;   % Tabela Simulador usa as pressões em Kgf/cm2, quando as medições do processo são em bar
    vazaoOleo_estimada = f_matrizVazao_sym(Freq,PChegada); % A predição é feita com base na Tabela do Simulador, 
    % Monta vetor para retornar a predição no instante seguinte
    predicoes = [predicoes; vazaoOleo_estimada];

end
%% ==============================================================================
%% Função para execução da predição da ESN um passo a frente
function [predicoes, novo_a0] = executa_predicao_ESN(EntradaU,EstadosX, ESNdataa0, modelo_ESN)
    % EntradaU são as entradas do processo no instante atual : frequencia_BCSS, pressao_montante_alvo
    % EstadosX são as 11 variáveis do processo no instante atual:

    % Observar que a ESN tem como entrada as 2 variáveis manipuladas e 10 variáveis do processo
    % uma vez que não trata a vazão
    
    % Assim, "entradas" na função é uma vetor coluna: frequencia_BCSS, pressao_montante_alvo, ...
    %           pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
    %           temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, ...
    %           temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
    %   
    % modelo_ESN precisar ter: .data.Wrr, .data.Wir, .data.Wbr
    %
    % ESNdataa0 é o estado atual do reservatorio da ESN
    %
    % Saidas: 
    %           Retorna um vetor coluna com a estimativa das 11 variáveis do processo:
    %           pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
    %           temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, ...
    %           temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
    % Retorna os estados do reservatório da ESN para atualizar

    entradas=[EntradaU;EstadosX(1:end-1)];   % Monta vetor para entrada da ESN
    entradas_normalizadas = normaliza_entradas(entradas); 
    x_ESN = modelo_ESN.data.Wrr*ESNdataa0 + modelo_ESN.data.Wir*entradas_normalizadas + modelo_ESN.data.Wbr;  %usar o modeloPreditor(ESN) para fazer a predição
    novo_a0 = (1-modelo_ESN.data.gama)*ESNdataa0 + modelo_ESN.data.gama*tanh(x_ESN);         % Atualiza estado da ESN
    a_wbias = [1.0; novo_a0];                                                                         % 
    predicoes_normalizadas = modelo_ESN.data.Wro*a_wbias;  

    % Retorna os valores dos novos estados preditos pela ESN com 1 passo a frente
    predicoes = desnormaliza_predicoes(predicoes_normalizadas);  
end