%%  IMPLEMENTAÇÃO MPC USANDO A BIBOIOTECA CASADI
classdef casadi_block_Control < matlab.System & matlab.system.mixin.Propagates
    properties (DiscreteState)
    end
    properties (Access = private)                       % Criação das variáveis que vão compor o OBJETO
        casadi_solver                                            % Criação do solver Casadi
        x0                                                                % Criação da variável para guardar as variáveis medidas (estados X) 
        u0                                                                % Criação da variável para guardar as ações de controle (entradas U)
        Predicao                                                     % Criação da variável para guardar a predição
        Funcao_Interpola                                       % Para carregar uma única vez a 'f_Interpola_casadi_vazao_sym' 

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
            num = 7;                                               % São 7 pinos
            %    11 variáveis do processo (10 +  vazão)
            %     2 manipuladas (Freq e PMonAlvo aplicadas ao processo)
            %     1  = clock = tempo de simulação dado pelo Simulink
            %     2  = Alvos = Valores alvo ENG para as manipuladas (Freq e PMon)
            %     1  = Hp = Horizonte de predição do controlador MPC
            %     1  = Hc = Horizonte de controle MPC
            %     1 = PassoMPC = Taxa de amstragem proporcional do MPC em relação aos dados de amostragem do processo

        end
        %===============        
        function sz1 = getInputSizeImpl(~)            % Organização das entradas do bloco Casadi
            sz1=7;
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

        end
        %===============        
        function sz1 = getOutputSizeImpl(~)         % Organização da saida do bloco Casadi
            % Feasible (Dim=1)
            % Iteracoes (Dim=1)
            % TempoSolver (Dim=1)
            % Ação de controle (Dim=nu)
            % DeltaU (Dim=nu)
            sz1 = 1+1+1+2+2;
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
 %% ================     Inicialização geral dos parâmetros e solver - só passa aqui uma única vez
       function setupImpl(obj,~)
            tInicializa=tic;                                         % Marcador para o tempo gasto no Solver
            import casadi.*                                      % Importação da biblioteca Casadi (tem de estar no path)
            obj.contador = 0;                                    % Contador para indicar passos/momentos de atuação do controle MPC
            
            %% Funções simbólicas que serão necessárias ao Solver
            f_Interpola_casadi_vazao_sym = evalin('base', 'f_Interpola_casadi_vazao_sym'); 
            f_buscaLimites_sym=evalin('base', 'f_buscaLimites_sym'); 
            obj.Funcao_Interpola=f_Interpola_casadi_vazao_sym;
            
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
         
            % Dos dados de operação
            umax =  evalin('base','umax');            % Valores máximos para as entradas (Freq e PMonAlvo)     
            umin =  evalin('base','umin');              % Valores mínimos para as entradas (Freq e PMonAlvo)     
            dumax =  evalin('base','dumax');          % Variação máxima do delta U nas entradas (variáveis manipuladas Freq e PMonAlvo)
            MargemPercentual=evalin('base','MargemPercentual');   % Margem de folga para os limites dos alarmes   
            
            %% Carrega condições inciais conhecidas apenas para inicializar simulação/operação
            x0=evalin('base','XIni');                       % Condições iniciais das variáveis (estados) do processo (em coluna)
            nx=height(x0);                                      % Número de variáveis (estados) do processo
            u0=evalin('base','UIni');                       % Condições iniciais das entradas (variáveis manipuladas) do processo (em coluna)
            nu=height(u0);                                      % Número de variáveis de entrada no processo
            obj.x0 = repmat(x0,(1+Hp),1);       % Condição das variáveis medidas (estados X) atuais e futuras
            obj.u0 = repmat(u0,Hc,1);             % Condição inicial para as ações de controle (U) atuais e futuras
                      
            %% Carrega matriz que faz o papel de y=h(x), onde a saída y(k) é função dos estados x(k)
            matriz_h=evalin('base','matriz_h');        % matriz para calcular as saidas na forma y=h(x)
            EstadosMedidos=MX.sym('EstadosMedidos',nx,1); 
            h=Function('h',{EstadosMedidos},{matriz_h*EstadosMedidos}); 

            %% Apenas para inicializar saidas controladas por setpoint
            y0 = h(x0);                                             % Valor das variáveis de saida (controladas por setpoint) na forma de coluna
            ny=height(y0);                                        % Numero de variáveis de saida (controladas por setpoint)
            
           %% Carrega modelo preditor e esquenta a ESN
            ModeloPreditor = evalin('base', 'ModeloPreditor');   % Modelos do preditor que será usada pelo MPC
%             TipoPreditor=ModeloPreditor.data.tipo;                    % Verifica tipo 1=ESN, 2 = LSTM (USAMOS APENAS ESN)
            entradas_normalizadas = normaliza_entradas([u0;x0]);   % Normaliza entradas provenientes do processo (observar que a função nada faz com a vazão)
            for i=1:1000     % Esquenta a ESN
               % Novos estados da ESN com base no estado atual e na taxa de vazamento
                Predicao = ModeloPreditor.data.Wrr*ModeloPreditor.data.a0 +  ModeloPreditor.data.Wir*entradas_normalizadas + ModeloPreditor.data.Wbr; 
                ModeloPreditor.data.a0 = (1-ModeloPreditor.data.gama)*ModeloPreditor.data.a0 + ModeloPreditor.data.gama*tanh(Predicao);
            end
            obj.ModeloPreditor = ModeloPreditor;           % Guarda como OBJ pois a ESN precisará ter seus estados internos atualizados a cada amostragem
            nx_ESN = length(ModeloPreditor.data.a0);  % Tamanho do reservatório da ESN para poder enviar o ModeloPreditor como parâmetro para o Solver
            [x_predito, ESNdataa0] = executa_predicao(u0,x0, ModeloPreditor.data.a0, ModeloPreditor, f_Interpola_casadi_vazao_sym);
           obj.Predicao=x_predito;                                  % Guarda predição atual
           
            %% Variáveis simbolicas para o problema de otimização
            X =MX.sym('X',1+Hp,nx);                      % Estado atual + estados futuros até Hp 
            U=MX.sym('U',Hp,nu);                          % Ações de controle até o horizonte Hp
            
            % Parâmetros que devem ser oferecidos ao  Solver
            % Estados estimados no instante anterior + Saidas desejadas (variáveis controladas por setpoint) +  Reservatório da ESN
            P =MX.sym('P',nx+ny+nx_ESN);                      
 
            %% ===================================================
            % Montagem das restrições (atuais e futuras) dos estados X (lbx/ubx) e
            % restrições "livres" que podem ser de igualdade ou desigualdade (lbg/ubg)
            args=struct;     % Inicializa variável que vai armazenar a estrutura de argumentos do NLP
            
            %% Restrições em X e U para o MPC (lbx e ubx) para o estado atual e estados futuros
            % Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo HP  +  Ações de controle em todo Hc ]
            %                             Dimensão = [        nx                      nx*Hp                                                 nu*Hc ]
            %
            % Observe que os estados X (medições) tem restrições dinâmicas, mas sabemos que lbx/ubx não podem conter fórmulas
            % Assim, deixaremos as restrições lbx/ubx "livres" para os estados X e as restrições serão efetivamente tratadas em g
            % No caso da ação de controle (Freq e PMonAlvo), apenas a Freq tem restrições fixas definitivas e as restrições da PMonAlvo 
            % são limitadas (por proteção), mas seus limites também serão tratados de forma dinâmica em g 

            args.lbx=[];       % Inicializa limites inferiores para as restrições dos estados X do MPC e ações de controle
            args.ubx=[];     % Inicializa limites superiores para as restrições dos estados X do MPC e ações de controle

            for i=1:1+Hp                     % Para os estados atuais e para todo o horizonte Hp
                args.lbx=[args.lbx, -inf(1,nx) ];          % Limites inferiores para as restrições de X      
                args.ubx=[args.ubx, inf(1,nx)];          % Limites superiores para as restrições de X
            end
            for i=1:Hc                                              % Para as ações de controle em todo o horizonte Hc
                args.lbx=[args.lbx, umin' ];                % Limites inferiores para as restrições U      
                args.ubx=[args.ubx, umax' ];             % Limites superiores para as restrições U
            end

            %% Montar agora as restrições de igualdade/desigualdade em g
            % Inicializa variável para guardar as fórmulas das restrições que vamos criar livremente
            % Neste caso, o formato que for criado também deve ser considerado para os valores em todo o horizonte futuro
            g=[];             
            args.lbg=[];       % Inicializa limites inferiores para as restrições que vamos criar "livremente"
            args.ubg=[];     % Inicializa limites superiores para as restrições que vamos criar "livremente"

            %% Referentes aos erros de predição
            % ERRO ZERO ESTÁ CORRETO??? DEVERÍAMOS CRIAR UMA TOLERÂNCIA PARA OS ERROS DE ESTIMAÇÃO ???
            xm=X(1,:);                                              % Medições atuais dos estados X
            x0=P(1:nx)';                                            % Estados estimados no instante anterior
            Ex= (xm-x0);                                          % Erro entre medições atuais dos estados X e valores estimados no instante anterior
%             g=[g;  Ex'];                                            % Empilha restrição de igualdade
%             args.lbg=[args.lbg     zeros(1,nx)];    % Zeros para os limites inferiores  
%             args.ubg=[args.ubg  zeros(1,nx)];    % Zeros para os limites superiores, caracterizando a restrição de igualdade (igual a zero)

            y0 = h(x0')';                                             % Valor das variáveis de saida (controladas por setpoint) estimadas no instante anterior
            ym = h(xm')';                                       % Medições das saidas controladas por setpoint 
            Ey= (ym-y0);                                         % Diferença entre as mediçoes e valores estimados no instante anterior
%             g=[g;  Ey'];                                                 % Empilha restrição de igualdade
%             args.lbg=[args.lbg     zeros(1,ny)];        % Zeros para os limites inferiores  
%             args.ubg=[args.ubg  zeros(1,ny)];         % Zeros para os limites superiores, caracterizando a restrição de igualdade (igual a zero)

            %% Restrições para todo o loop de predição
            fob=0;                                                            % Inicializa custo da função objetivo
            Ysp=P(nx+1:nx+ny)';                                     % Extrai os setpoints recebidos via parâmetros
            ModeloPreditor.data.a0=P(nx+ny+1:end);  % Extrai reservatório da ESN enviados para o Solver via parâmetros  
            for k=1:Hp
                EstadoAtual=X(k,:);                                   % Medições (estados) no instante k atual até horizonte Hp
%                 EstadoAtual=X(k+1,:);                                   % Medições (estados) no instante k atual até horizonte Hp
                AcaoAtual=U(k,:);                                      % Ação de controle k atual até horizonte Hp
                [x_predito, ESNdataa0] = executa_predicao(AcaoAtual',EstadoAtual', ModeloPreditor.data.a0, ModeloPreditor, f_Interpola_casadi_vazao_sym);
                ModeloPreditor.data.a0=ESNdataa0;     % Mantem a ESN com reservatório atualizado durante o loop de predição
                y_predito= h(x_predito)';                            % Saida (variáveis controladas por setpoint)
                x_predito=x_predito';

                % Insere restrições dinâmicas para serem tratadas no loop
                LimitesX= f_buscaLimites_sym(AcaoAtual(1));   % Resgata limites de alarmes para as variáveis do processo em função da frequência
                LimitesY=matriz_h*LimitesX';                                 % Extrai limites correspondentes as saidas (variáveis controladas por setpoint)

                LimitesX(1,:)=LimitesX(1,:)*(1-MargemPercentual/100);   % Implementa margem de folga em relação ao máximo
                LimitesX(2,:)=LimitesX(2,:)*(1+MargemPercentual/100);   % Implementa margem de folga em relação ao mínimo
                
                % RESTRIÇÕES PARA AS VARIÁVEIS DO PROCESSO (ESTADOS X)
                % Insere restrições para os valores máximos das variáveis (estados X) preditos
                LimMaxX=LimitesX(1,:)-x_predito;        % Para não ser violado o limite, a diferença deve ser >= 0
                g=[g; LimMaxX']; 
                args.lbg=[args.lbg,  zeros(1,nx) ];      % Limite mínimo para restrição de desigualdade      
                args.ubg=[args.ubg, inf(1,nx)];       % Limite máximo para restrição de desigualdade
                % Insere restrições para os valores mínimos das variáveis (estados X) preditos
                LimMinX=x_predito-LimitesX(2,:);         % Para não ser violado o limite, a diferença deve ser >= 0
                g=[g; LimMinX']; 
                args.lbg=[args.lbg,   zeros(1,nx) ];        % Limite mínimo para restrição de desigualdade      
                args.ubg=[args.ubg,  inf(1,nx) ];       % Limite máximo para restrição de desigualdade
                
                % RESTRIÇÕES PARA AS VARIÁVEIS DE SAIDA (CONTROLADAS POR SETPOINT)
                % Insere restrições para os valores máximos das saidas preditas
                LimMaxY=LimitesY(1,:)-y_predito;    % Para não ser violado o limite, a diferença deve ser >= 0
                g=[g; LimMaxY']; 
                args.lbg=[args.lbg,  zeros(1,ny) ];      % Limite mínimo para restrição de desigualdade      
                args.ubg=[args.ubg, inf(1,ny) ];          % Limite máximo para restrição de desigualdade
                % Insere restrições para os valores máximos das saidas preditas
                LimMinY=y_predito-LimitesY(2,:);      % Para não ser violado o limite, a diferença deve ser >= 0
                g=[g; LimMinY']; 
                args.lbg=[args.lbg, zeros(1,ny) ];        % Limite mínimo para restrição de desigualdade      
                args.ubg=[args.ubg,inf(1,ny) ];             % Limite máximo para restrição de desigualdade

                ErroEstimativaX=x_predito-EstadoAtual;  % Erro da predição dos estados
                
               fob=fob+(y_predito-Ysp+Ey)*Qy*(y_predito-Ysp+Ey)' + (ErroEstimativaX+Ex)*Qx*(ErroEstimativaX+Ex)';
            end
             
            for k=1:Hc
                fob=fob+U(k,:)*Qu*U(k,:)';        % Insere custos das ações de controle U (atuais e futuras) na fob
                g=[g; U(k,:)'];                              % Insere restrição de desigualdade
                args.lbg=[args.lbg,    umin' ];     % Limite mínimo para a restrição de desigualdade associada a ação de controle U      
                args.ubg=[args.ubg,  umax' ];   % Limite máximo para restrição de desigualdade associada a ação de controle U
            end
            
            for k=1:Hc-1
                DeltaU=U(k+1,:)-U(k,:);            % Calcula DeltaU
                fob=fob+DeltaU*R*DeltaU';     % Insere custo de DeltaU (atuais e futuros) na fob
                g=[g; DeltaU'];                            % Insere restrição de desigualdade
                args.lbg=[args.lbg,   -dumax' ];  % Limite mínimo para a restrição de desigualdade associada ao DeltaU      
                args.ubg=[args.ubg,  dumax' ];  % Limite máximo para restrição de desigualdade associada ao DeltaU
            end
            
            obj.lbx=args.lbx;                                % Lower Bounds para os Estados X e U do MPC
            obj.ubx=args.ubx;                             % Upper Bounds para os Estados X e U do MPC
            obj.lbg=args.lbg;                                % Lower Bounds para as restrições [g] que foram criadas
            obj.ubg=args.ubg;                             % Upper Bounds para as restrições [g] que foram criadas

            %% ========================Configuração do otimizador====================================
            % Monta as variáveis de decisão em um vetor coluna - esta dimensão é fundamental para entender as contas e  indexações
            % Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo Hp  +  Ações de controle em todo Hc ]
            %                             Dimensão = [                              nx*(1+Hp)                    ;        nu*Hc ]
            opt_variables=[reshape(X,nx*(Hp+1),1)  ;   reshape(U,nu*Hc,1)];

            nlp = struct('f',fob,'x',opt_variables,'g', g, 'p', P); % Define a estrutura para problema de otimização não linear (NLP, Nonlinear Programming)
            
            %Configuração específica do IPOPT
            options=struct;
            options.print_time=0;                           % 
            options.ipopt.print_level=0;                  % [ 0 a 12] = (funciona 3 a 12) Detalhe do nivel de informação para mostrar na tela durante a execução do solver
            options.ipopt.bound_relax_factor=0;    % Tolerância absoluta para as restrições definidas pelo usuário (default=1e-8)
            
            options.ipopt.max_iter=100;                   % Especifica o número máximo de iterações que o solver deve executar antes de parar.
            options.ipopt.max_wall_time=0.4;           % Tempo (em segundos) máximo para solver encontrar solução
         
            solver = nlpsol('solver','ipopt', nlp,options); % Define o Interior Point OPTimizer (ipopt) para resolver o problema de otimização não linear (nlp)            
            obj.casadi_solver=solver;
            t_inicializacao = toc(tInicializa);           % Tempo gasto para a inicialização do Solver
            disp(strcat("Tempo para inicialização = ",num2str(t_inicializacao)))
     
       end
        
%% ================  Contas de atualização -  Equivale a Flag=2 na SFunction)
        function  SaidaMPC= stepImpl(obj,DadosProcesso,AcaoDeControle,t,Ysp,PassoMPC,Hp,Hc)
            import casadi.*                                      % Importação da biblioteca Casadi (tem de estar no path)

            args=struct;     % Inicializa variável que vai armazenar a estrutura de argumentos

            disp(strcat("Simulação MPC em ",num2str(t)," s"))

            %% Atualiza condição inicial para o Solver
            nx=height(DadosProcesso);                 % Extrai a dimensão em função do número de variáveis em DadosProcesso
            nu=height(AcaoDeControle);                % Extrai a dimensão em função do número de variáveis nas Ações de controle
            ny=height(Ysp);                                      % Extrai a dimensão em função do número de variáveis recebidas com SetPoint
            
            % Inicialização para um novo passo do Solver
            % Inicializar com valores preditos deve diminuir o tempo de busca do solver
            X0=ShiftDown(obj.x0,DadosProcesso);   % Atualza condição inicial dos estados com a medição atual e valores antes preditos
            U0=ShiftDown(obj.u0,AcaoDeControle); % Atualza condição inicial das ações de controle com ação atual e valores antes propostos
            DeltaU=zeros(nu,1);                                  % Assume DeltaU=0, mas pode alterado se passar pelo Solver
            args.x0=[ X0;  U0];                                     % Estados X e ações de controle U, atuais e futuras, como condição inicial para passar ao solver

           %% Resgata restrições guardadas pelo objeto
            args.lbx=obj.lbx;                                             % Lower Bounds para os Estados X e U do MPC
            args.ubx=obj.ubx;                                          % Upper Bounds para os Estados X e U do MPC
            args.lbg=obj.lbg;                                             % Lower Bounds para as restrições [g] que foram criadas
            args.ubg=obj.ubg;                                          % Upper Bounds para as restrições [g] que foram criadas

           %% Atualiza parâmetros que precisam ser enviados ao Solver
            args.p=[full(obj.Predicao); Ysp; obj.ModeloPreditor.data.a0];          % Predição anterior, Setpoint das variáveis desejadas e reservatório ESN entram p/ o Solver na forma de parâmetros

            %% ===================== %Parâmetros e atuação do solver ========================================
            obj.contador = obj.contador+1;     % Contador ajudará a saber se é para o Solver do Otimizador/Controlador atuar
            TempoSolver=0;                            % Inicializa contados para calcular o tempo gasto com o Solver, quando for o caso !!
            Feasible=0.5;                                 % Assumir padrão para indicar que não passou pelo Solver
            Iteracoes=0;                                   % Numero de iterações, alterado qdo passa pelo Solver
            if (obj.contador==PassoMPC)      % Solver só entra no passo definido pelos parâmetros de Passo do MPC
                TempoIni=tic;
                solver=obj.casadi_solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
                Feasible=obj.casadi_solver.stats.success;
                Iteracoes=obj.casadi_solver.stats.iter_count;
%                 % Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo Hp  +  Ações de controle em todo Hc ]
                Solucao_MPC=full(solver.x);                             % Solução ótima encontrada pelo otimizador
                X0=Solucao_MPC(1:nx*(1+Hp));                % Assume nova condição inicial para os estados X atuais e preditos 
                NewU0=Solucao_MPC(nx*(1+Hp)+1:end); % Resgata novas ações de controle ótimas calculadas 
                DeltaU=NewU0(1:nu)-U0(1:nu);                  % Registra variação DeltaU
                U0=NewU0;                                                  % Assume nova condição inicial para as ações U, atuais e preditas 
                obj.contador = 0;                                            % Reinicia contador para a atuação do MPC
                TempoSolver = toc(TempoIni);                     % Tempo gasto pelo Solver
            end
            U=U0(1:nu,1);                                                       % Extrai especificamente as ações de controle para serem aplicadas no processo
            SaidaMPC=[Feasible, Iteracoes, TempoSolver, U', DeltaU']';
            
           % Para atualizar o modelo preditor é necessário manter a ESN atualizada
            ModeloPreditor=obj.ModeloPreditor;
            UProcesso=U0(1:nu);                          % Como não pode aplicar em todo o Hc, seleciona a primeira ação
            entradas_normalizadas = normaliza_entradas([UProcesso;DadosProcesso]);   % Normaliza entradas provenientes do processo (observar que a função nada faz com a vazão)
            [x_predito, ESNdataa0] = executa_predicao(UProcesso,DadosProcesso, ModeloPreditor.data.a0, ModeloPreditor, obj.Funcao_Interpola);
            ModeloPreditor.data.a0=ESNdataa0;     % Mantem a ESN com reservatório atualizado durante o loop de predição
            obj.ModeloPreditor = ModeloPreditor;
            obj.Predicao=full(x_predito);
        end
    end
end

%===============================================================================
% ==================  FIM DO PROGRAMA PRINCIPAL  ================================
%===============================================================================

%% ==================        Outras funções adicionais   ================================
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

    entradas=[EntradaU;EstadosX(1:end-1)];   % Monta vetor para entrada da ESN (exclui último = vazão)
    entradas_normalizadas = normaliza_entradas(entradas); 
    x_ESN = modelo_ESN.data.Wrr*ESNdataa0 + modelo_ESN.data.Wir*entradas_normalizadas + modelo_ESN.data.Wbr;  %usar o modeloPreditor(ESN) para fazer a predição
    novo_a0 = (1-modelo_ESN.data.gama)*ESNdataa0 + modelo_ESN.data.gama*tanh(x_ESN);         % Atualiza estado da ESN
    a_wbias = [1.0; novo_a0];                                                                         % 
    predicoes_normalizadas = modelo_ESN.data.Wro*a_wbias;  

    % Retorna os valores dos novos estados preditos pela ESN com 1 passo a frente
    predicoes = desnormaliza_predicoes(predicoes_normalizadas);  
end
%% ==============================================================================
%% Função para atualizar estados iniciais com dados recebidos do processo (vetores coluna)
function S=ShiftDown(DadosOriginais,NovosDados);
    Tam=height(NovosDados);                                    % Tamanho do vetor com novos dados à serem inseridos
    S=[NovosDados;DadosOriginais(1:end-Tam)];    % Insere novos dados no inicio e desloca os DadosOriginais para baixo
end
%% ==============================================================================
