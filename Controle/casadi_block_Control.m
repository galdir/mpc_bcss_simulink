%%  IMPLEMENTAÇÃO MPC USANDO A BIBOIOTECA CASADI
classdef casadi_block_Control < matlab.System & matlab.system.mixin.Propagates
    properties (DiscreteState)
    end
    properties (Access = private)                       % Criação das variáveis que vão compor o OBJETO
        casadi_solver                                            % Criação do solver Casadi
        % Variáveis que representarão a consição inicial para o solver em cada operação do Solver
        x0                                                                % Criação da variável para guardar as variáveis medidas (estados X) atuais e em todo horizonte (1+Hp) 
        du0                                                              % Criação da variável para guardar as ações de controle (DeltaU) em todo o horizonte (Hc)
        ysp0                                                            % Criação da variável para guardar os setpoints ótimos calculados para as variáveis controladas por setpoint
        
        Hp                                                               % Horizonte de predição
        Hc                                                                % Horizonte de controle
        nx                                                                 % Numero de variáveis de entrada (estados) do processo
        ny                                                                 % Numero de variáveis (saida do processo) que são controladas por setpoint
        nu                                                                 % Numero de variáveis manipuladas (ações de controle possíves) - no caso, Freq e PMonAlvo
        PassoMPC                                                  % Numero de amostragens até a atuação do MPC
        
        Predicao                                                     % Criação da variável para guardar a predição
        ModeloPreditor                                           % Criação da variável para guardar modelo de preditor do processo e que será utilizada pelo solver para a predição
        EstimaVazao                                               % Para carregar uma única vez a 'f_Interpola_casadi_vazao_sym' 
        Funcao_h                                                    % Para procedes a conta y=h(x) e obter as saidas em função da matriz h definida
        
        lbx                                                                % Lower Bounds para os Estados do MPC
        ubx                                                               % Upper Bounds para os Estados do MPC
        lbg                                                                % Lower Bounds para as restrições [g] que forem criadas
        ubg                                                               % Upper Bounds para as restrições [g] que forem criadas
        
        contador                                                     % Variável para guardar o contador de passos de amostragem - usado para definir momentos de atuação do MPC
    end
%%======================================================
    methods (Access = protected)
        %% ============   Ajuste das entradas
        function num = getNumInputsImpl(~)        % Nùmero de pinos de entradas do bloco CASADI (ver no Simulink)
            num = 3;                                               % São 3 pinos
            %    11 variáveis do processo (10 +  vazão)
            %     2 manipuladas (Freq e PMonAlvo aplicadas ao processo)
            %     2  = Alvos = Valores alvo ENG para as manipuladas (Freq e PMon)

        end
        %===============        
        function sz1 = getInputSizeImpl(~)            % Organização das entradas do bloco Casadi
            sz1=3;
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
            % Ação de controle (Dim=nu)               % Ação na Freq e PMonAlvo
            % DeltaU (Dim=nu)                                % DeltaU na Freq e PMonAlvo
            % AlvoOtimoCalculado (Dim=nu)                           % Alvos Ótimos  calculados pelo Solver para Freq e PMonAlvo

            % Carrega matrizes apenas para extrair a dimensão das variáveis
            Qx= evalin('base','Qx');    nx=height(Qx);  % Número de variáveis (estados) do processo
            Qu = evalin('base','Qu');  nu=height(Qu);  % Número de variáveis de entrada no processo (manipuladas)
            Qy=  evalin('base','Qy');   ny=height(Qy);  % % Número de variáveis de saida controladas por SetPoint

            sz1 = 1+1+1+nu+nu+nu;
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
            tInicializa=tic;                                         % Marcador para o tempo gasto na Inicialização
            import casadi.*                                      % Importação da biblioteca Casadi (tem de estar no path)
            obj.contador = 0;                                    % Contador para indicar passos/momentos de atuação do controle MPC
                  
            %% Funções simbólicas que serão necessárias ao Solver
            f_Interpola_casadi_vazao_sym = evalin('base', 'f_Interpola_casadi_vazao_sym'); 
            f_buscaLimites_sym=evalin('base', 'f_buscaLimites_sym'); 
            obj.EstimaVazao=f_Interpola_casadi_vazao_sym;                  % Guarda pois a função será necessária durante a execução
            
            %% Carrega tabelas Petrobras
            % Tabelas para cálculos das proteções dinâmicas
            % Transforma tabelas em matrizes para melhorar desempenho computacional e permitir o uso de algébra simbólica (CaSAdi)
            TabelaLimitesDinamicos =evalin('base', 'TabelaLimitesDinamicos');       % Tabela com limites Max/Min de todas as variáveis em todas as frequências (com resolução de 0,1)
            MatrizLimitesDinamicos = table2array(TabelaLimitesDinamicos);

            % Tabela do Simulador para cálculos da vazão por interpolação
            TabelaSimulador=evalin('base', 'TabSimulador');                               % Tabela do Simulador para cálculos da vazão
            MatrizSimuladorVazao = table2array(TabelaSimulador(:,1:3));        % Especificamente as colunas Freq, PChegada e Vazao para diminuir o esforço computacional
            
            %% Carrega parâmetros definidos pelo usuário
            % Do controlador
            Hp =  evalin('base','Hp');                     % Horizonte de predição
            obj.Hp=Hp;
            
            Hc =  evalin('base','Hc');                     % Horizonte de controle
            obj.Hc=Hc;
            
            PassoMPC=evalin('base','PassoMPC');    % Quantos passos de amostragem para a atuação do MPC
            obj.PassoMPC=PassoMPC;
            
            Qx= evalin('base','Qx');                       % Ponderação para os erros de estimação das variáveis do processo
            Qu = evalin('base','Qu');                      % Ponderação das ações de controle nas entradas (Alvos Desejados = Freq. e PMonAlvo)
            Qy=  evalin('base','Qy');                      % Ponderação das saidas controladas por setpoint
            R = evalin('base','R');                          % Ponderação das ações de controle nas entradas (Alvos Desejados = Freq. e PMonAlvo)
            
            nx=height(Qx);   % Número de variáveis (X estados) do processo (11 = 10+ vazão)
            obj.nx=nx;
            nu=height(Qu);  % Número de variáveis de entrada no processo (U manipuladas) = Freq e PMonAlvo
            obj.nu=nu;
            ny=height(Qy);  % % Número de variáveis de saida (Y controladas por setpoint) = PChegada e Vazão
            obj.ny=ny;

            % Dos dados de operação
            umax =  evalin('base','umax');            % Valores máximos para as entradas (Freq e PMonAlvo)     
            umin =  evalin('base','umin');               % Valores mínimos para as entradas (Freq e PMonAlvo)     
            dumax =  evalin('base','dumax');          % Variação máxima do DeltaU nas entradas (variáveis manipuladas Freq e PMonAlvo)
            MargemPercentual=evalin('base','MargemPercentual');   % Margem de folga para os limites dos alarmes   
            
            %% Carrega matriz que faz o papel de y=h(x), onde a saída y(k) é função dos estados x(k)
            matriz_h=evalin('base','matriz_h');        % Matriz para calcular as saidas do processo e que são controladas por setpoint na forma y=h(x)
            EstadosMedidos=MX.sym('EstadosMedidos',nx,1); 
            h=Function('h',{EstadosMedidos},{matriz_h*EstadosMedidos}); 
            obj.Funcao_h=h;                                     % Guarda função pois será requerida na simulação

            %% Carrega condições inciais conhecidas apenas para inicializar simulação
            x0=evalin('base','XIni');                       % Condições iniciais das variáveis (estados) do processo (em coluna)
            y0 = h(x0);                                             % Valor das variáveis de saida do processo (Y controladas por setpoint) na forma de coluna
            u0=evalin('base','UIni');                       % Condições iniciais das entradas (variáveis manipuladas U) do processo (em coluna)
                     
           %% Carrega modelo preditor e esquenta a ESN
            ModeloPreditor = evalin('base', 'ModeloPreditor');   % Modelos do preditor que será usada pelo MPC
%        TipoPreditor=ModeloPreditor.data.tipo;                    % Verifica tipo 1=ESN, 2 = LSTM (USAMOS APENAS ESN)
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
           

           % Condições inciais para a estrutura de estados do MPC
           obj.x0 = repmat(x0,(1+Hp),1);              % Condição das variáveis medidas (estados X) atuais e futuras
           obj.ysp0=repmat(y0,(1+Hp),1);            % Inicializa Ysp atuais e futuros com os valores atuais na saida da planta
           du0=zeros(Hc,nu);                                 % Estrutura para armazenar DeltaU atuais e futuros, até o horizonte Hc 
           obj.du0=du0(:);                                       % Condição inicial para os DeltaU em todo o horizonte futuro
            
           %% =============================================================================================
           %  Até aqui foi a inicialização das variáveis e estruturas, salvando em OBJ para que possam ser usadas no StepImpl 
           % Doravante precisamos tratar tudo de forma simbólica para o Solver
           %% =============================================================================================
            %% Variáveis simbolicas para o problema de otimização (as linhas representarão a evolução do instantes k)
            % Variáveis de decisão - são para elas que criaremos as restrições lbx/ubx
            X =MX.sym('X',1+Hp,nx);                      % Estado atual + Estados futuros até Hp 
            DU = MX.sym('DU',Hp,nu);                   % Incrementos do controle sobre o horizonte até Hp (precisa chegar ao Hp, mesmo que no custo use só até Hc)
            Ysp = MX.sym('Ysp',1+Hp,ny);              % Set-point otimo calculado pelo otimizador (Variável de decisão) atual e todo Hp
            U = MX.sym('U',Hp,nu);                         % Temporária para termos as ações de controle no horizonte até Hp
            
            % Parâmetros que devem ser oferecidos para o Solver
            %         EntradaAtual   Ação atual + AlvoENG + ErroX  + ErroY + Reservatório da ESN
            Indice=[  nx                        nu               nu            nx             ny                nx_ESN];
            % Cria vetor de parâmetros na dimensão especificada
            P =MX.sym('P',sum(Indice));    
            % Associa variáveis simbólicas as respectivas partes no vetor de parâmetros
            [ Xk,Uk, AlvoEng,ErroX, ErroY, Reservatorio_ESN]=ExtraiParametros(P,Indice);
            ModeloPreditor.data.a0=Reservatorio_ESN;      % Resgata estado atual do reservatório da ESN
            %% ===================================================
            % Montagem das restrições (atuais e futuras) dos estados X (lbx/ubx) e
            % restrições "livres" que podem ser de igualdade ou desigualdade (lbg/ubg)
            args=struct;     % Inicializa variável que vai armazenar a estrutura de argumentos do NLP
            args.lbx=[];       % Inicializa limites inferiores para as restrições dos estados X do MPC e ações de controle
            args.ubx=[];     % Inicializa limites superiores para as restrições dos estados X do MPC e ações de controle
            args=Monta_lbx_ubx(args,Hp,Hc,nx,ny,dumax);

           %% Montar agora as restrições de igualdade/desigualdade em g
            % Inicializa variável para guardar as fórmulas das restrições que vamos criar livremente
            % OBSERVAR QUE AS RESTRIÇÕES SÃO EMPILHADAS EM COLUNAS ENQUANTO SEUS
            % ARGUMENTOS (lbx/ubx, lbg/ubg) SÃO MONTADOS EM LINHA
            g=[];             
            args.lbg=[];       % Inicializa limites inferiores para as restrições que vamos criar "livremente"
            args.ubg=[];     % Inicializa limites superiores para as restrições que vamos criar "livremente"

            %% Montando o custo da função objetivo e restrições
            fob=0;                                                           % Inicializa custo da função objetivo
            fob=fob+ErroX'*Qx*ErroX;                          % Incrementa custo do erro de predição dos estados X
            X(1,:)=Xk';                                                     % Formata entradas atuais do processo para entrar no loop de predição
            Uk=Uk';                                                         % Formata ação de controle atual para entrar no loop de predição
            % Para todo o horizonte de predição futura
            for k=1:Hp
                Uk=Uk+DU(k,:);                                        % Nova ação de controle com base na ação atual e no DeltaU proposto
                U(k,:)=Uk;                                                  % Guarda temporariamente para outros cálculos que virão
                [x_predito, ESNdataa0] = executa_predicao(Uk',X(k,:)', ModeloPreditor.data.a0, ModeloPreditor, f_Interpola_casadi_vazao_sym);
                ModeloPreditor.data.a0=ESNdataa0;     % Mantem a ESN com reservatório atualizado durante o loop de predição
                y_predito= h(x_predito)';                           % Saida (variáveis controladas por setpoint)
                x_predito=x_predito';
                X(k+1,:)=x_predito;
                
               fob=fob+(y_predito-Ysp(k,:)+ErroY')*Qy*(y_predito-Ysp(k,:)+ErroY')';   % Incrementa função objetivo com erro entre as predições (atuais e futuras) 
               % O Ysp é uma variável de decisão para todo o horizonte de predição e o erro de estimador da saida (ErroY) é único em todo
               % o horizonte de predição. Tudo isso ponderado pela matriz Qy
               
                % APROVEITA O LOOP PARA CRIAR RESTRIÇÕES [g] EM TODO O HORIZONTE Hp
               
                % Calcula restrições dinâmicas para serem tratadas no loop
                LimitesX= f_buscaLimites_sym(Uk(1));      % Resgata limites de alarmes para as variáveis do processo em função da frequência
                LimitesY=matriz_h*LimitesX';                       % Extrai limites correspondentes as saidas (variáveis controladas por setpoint)

                LimitesX(1,:)=LimitesX(1,:)*(1-MargemPercentual/100);   % Implementa margem de folga em relação ao máximo
                LimitesX(2,:)=LimitesX(2,:)*(1+MargemPercentual/100);   % Implementa margem de folga em relação ao mínimo
                
                % RESTRIÇÕES PARA AS VARIÁVEIS DO PROCESSO (ESTADOS X)
                % Insere restrições para os valores máximos das variáveis (estados X) preditos
                LimMaxX=LimitesX(1,:)-x_predito;        % Para não ser violado o limite, a diferença deve ser >= 0
                g=[g; LimMaxX']; 
                args.lbg=[args.lbg,  zeros(1,nx) ];     % Limite mínimo para restrição de desigualdade      
                args.ubg=[args.ubg, inf(1,nx)];          % Limite máximo para restrição de desigualdade
                % Insere restrições para os valores mínimos das variáveis (estados X) preditos
                LimMinX=x_predito-LimitesX(2,:);    % Para não ser violado o limite, a diferença deve ser >= 0
                g=[g; LimMinX']; 
                args.lbg=[args.lbg,   zeros(1,nx) ];    % Limite mínimo para restrição de desigualdade      
                args.ubg=[args.ubg,  inf(1,nx) ];       % Limite máximo para restrição de desigualdade
                
                % RESTRIÇÕES PARA AS VARIÁVEIS DE SAIDA (CONTROLADAS POR SETPOINT)
                % Insere restrições para os valores máximos das saidas controladas por setpoint preditas
                LimMaxY=LimitesY(1,:)-y_predito;    % Para não ser violado o limite, a diferença deve ser >= 0
                g=[g; LimMaxY']; 
                args.lbg=[args.lbg,  zeros(1,ny) ];      % Limite mínimo para restrição de desigualdade      
                args.ubg=[args.ubg, inf(1,ny) ];          % Limite máximo para restrição de desigualdade
                
                % Insere restrições para os valores máximos das saidas controladas por setpoint preditas
                LimMinY=y_predito-LimitesY(2,:);      % Para não ser violado o limite, a diferença deve ser >= 0
                g=[g; LimMinY']; 
                args.lbg=[args.lbg, zeros(1,ny) ];         % Limite mínimo para restrição de desigualdade      
                args.ubg=[args.ubg,inf(1,ny) ];             % Limite máximo para restrição de desigualdade
            end
            
            % Restrições e custo da função objetivo na ação de controle (Freq e PMonAlvo)
             for k=1:Hc
                fob=fob+DU(k,:)*R*DU(k,:)';    % Insere na fob o custo de DeltaU (atuais e futuros até o Hc)
                g=[g; DU(k,:)'];                           % Insere restrição de desigualdade
                args.lbg=[args.lbg,   -dumax ];  % Limite mínimo para a restrição de desigualdade associada ao DeltaU      
                args.ubg=[args.ubg,  dumax ];  % Limite máximo para restrição de desigualdade associada ao DeltaU

                ErroU=U(k,:)-AlvoEng';               % Diferença entre a ação de controle e os alvos enviados pela engenharia
                fob=fob+ErroU*Qu*ErroU';        % Insere custos das ações de controle U (atuais e futuras) na fob

                % Restrições para a Frequencia
                g=[g; U(k,1)];                                 % Insere restrição de desigualdade especificamente para a Frequencia
                args.lbg=[args.lbg,    umin(1) ];    % Limite mínimo para a frequência      
                args.ubg=[args.ubg,  umax(1) ];   % Limite máximo para a frequência

                % Para as restrições da entrada PMonAlvo, serão também  consideradas as restrições dinâmicas da PChegada
                LimitesX= f_buscaLimites_sym(U(k,:));   % Resgata limites de alarmes para as variáveis do processo em função da frequência
                LimitesX(1,:)=LimitesX(1,:)*(1-MargemPercentual/100);   % Implementa margem de folga em relação ao máximo
                LimitesX(2,:)=LimitesX(2,:)*(1+MargemPercentual/100);   % Implementa margem de folga em relação ao mínimo
                % Olhando especificamente os limites da PMonAlvo, assume o limite mais conservador
                LimiteMinPmonAlvo=max(LimitesX(2,2),umin(2)); 
                LimiteMaxPmonAlvo=min(LimitesX(1,2),umax(2));

                g=[g; U(k,2)];                                           % Insere restrição de desigualdade especificamente na PMonAlvo
                args.lbg=[args.lbg,    LimiteMinPmonAlvo ];    % Limite mínimo para PMonAlvo considerados os limites dinâmicos da PChegada      
                args.ubg=[args.ubg, LimiteMaxPmonAlvo ];   % Limite máximo para PMonAlvo considerados os limites dinâmicos da PChegada
            end
                       
            obj.lbx=args.lbx;                                % Lower Bounds para os Estados X e U do MPC
            obj.ubx=args.ubx;                             % Upper Bounds para os Estados X e U do MPC
            obj.lbg=args.lbg;                                % Lower Bounds para as restrições [g] que foram criadas
            obj.ubg=args.ubg;                             % Upper Bounds para as restrições [g] que foram criadas

            %% ========================Configuração do otimizador====================================
            % Monta as variáveis de decisão em um vetor coluna - esta dimensão é fundamental para entender as contas e  indexações
            % Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo HP  +     DeltaU       +              Ysp          ]
            %                             Dimensão = [                  nx*(1+Hp)                                                     nu*Hc                    ny*(1+Hp)  ]
            du = DU(1:Hc,:);                                % Extrai Delta U apenas na parte correspondente ao Hc estabelecido
            opt_variables=[X(:); du(:) ; Ysp(:) ];

            nlp = struct('f',fob,'x',opt_variables,'g', g, 'p', P); % Define a estrutura para problema de otimização não linear (NLP, Nonlinear Programming)
            
            %Configuração específica do IPOPT
            options=struct;
            options.print_time=0;                           % 
            options.ipopt.print_level=0;                  % [ 0 a 12] = (funciona 3 a 12) Detalhe do nivel de informação para mostrar na tela durante a execução do solver
            options.ipopt.bound_relax_factor=0;    % Tolerância absoluta para as restrições definidas pelo usuário (default=1e-8)
            
            options.ipopt.max_iter=100;                   % Especifica o número máximo de iterações que o solver deve executar antes de parar.
%             options.ipopt.max_wall_time=0.4;           % Tempo (em segundos) máximo para solver encontrar solução
            options.ipopt.max_wall_time=8;           % Tempo (em segundos) máximo para solver encontrar solução
         
            solver = nlpsol('solver','ipopt', nlp,options); % Define o Interior Point OPTimizer (ipopt) para resolver o problema de otimização não linear (nlp)            
            obj.casadi_solver=solver;
            t_inicializacao = toc(tInicializa);           % Tempo gasto para a inicialização do Solver
            disp(strcat("Tempo para inicialização = ",num2str(t_inicializacao)))
     
       end
        
%% ================  Contas de atualização -  Equivale a Flag=2 na SFunction)
        function  SaidaMPC= stepImpl(obj,DadosProcesso,Uk,AlvoEng)
%             disp(strcat("Simulação MPC em ",num2str(t)," s"))
            import casadi.*                                      % Importação da biblioteca Casadi (tem de estar no path)

            h=obj.Funcao_h;    % Restaura função para fazer o cálculo das saidas:  y=h(x);
            
            %% Dimensões para facilitar a indexação no código
            nx=obj.nx;                 % Extrai o número de variáveis em DadosProcesso
            nu=obj.nu;                 % Extrai o número de variáveis nas Ações de controle
            ny=obj.ny;                 % Extrai o numero de variáveis de saida (controladas por setpoint)
            Hp=obj.Hp;               % Tamanho do horizonte de predição
            Hc=obj.Hc;                % Tamanho do horizonte de controle
            PassoMPC=obj.PassoMPC;
            
            XPredito=obj.Predicao;                         % Predição dos estados feita no instante anterior
            ErroX=DadosProcesso-XPredito;       % Erro entre as medições atuais e a predição feita no instante anterior

            SaidasProcesso=h(DadosProcesso); % Saidas controladas por setpoint        
            YPredito=h(XPredito);       % Predição das saidas controladas por setpoint no instante anterior
            ErroY=SaidasProcesso-YPredito;       % Erro entre as medições atuais e a predição feita no instante anterior
         
            
            % Inicialização para um novo passo do Solver com base nos novos estados (entradas) medidos do processo
            % De uma forma geral, inicializar com valores atuais e toda a predição já feita antes, deve diminuir o tempo de busca do solver
            x0=ShiftDown(obj.x0,DadosProcesso);  % Atualiza condição inicial dos estados com a medição atual e valores passados
            du0=obj.du0;                                             % Resgata condição anterior do  DeltaU antes calculado
            ysp0=obj.ysp0;                                          % Resgata Setpoint das variáveis controladas por setpoint
            
                     
            %% ===================== %Parâmetros e atuação do solver ========================================
            obj.contador = obj.contador+1;     % Contador ajudará a saber se é momento para atuar o controlador 
            DeltaU=zeros(nu,1);                     % Reinicia DeltaU=0, inclusive para saber quando não passou pelo Solver
            TempoSolver=0;                            % Inicializa contados para calcular o tempo gasto com o Solver, quando for o caso !!
            Feasible=0.5;                                 % Assumir padrão para indicar que não passou pelo Solver
            Iteracoes=0;                                   % Numero de iterações, alterado qdo passa pelo Solver
            %% Passo para atuação do MPC/Solver
            if (obj.contador==PassoMPC)      % Solver só entra no passo definido pelos parâmetros de Passo do MPC
                TempoIni=tic;   % Inicaliza contagem de tempo para o Solver
                args=struct;     % Inicializa variável que vai armazenar a estrutura de argumentos para o solver

                %% Condição inicial para passar ao solver (inclui variáveis de decisão)
                % Atualiza setpoint das variáveis de saida em função dos novos Alvos da Engenharia
                Ysp = [ AlvoEng(2) ;   full(obj.EstimaVazao(AlvoEng(1),AlvoEng(2))) ];      % Setpoint para PChegada é a PMonAlvoENG e para Vazao é estimada em função da FreqAlvo e PMonAlvo
                ysp0=ShiftDown(obj.ysp0, Ysp);              % Atualiza condição inicial dos dos setpoints das variáveis de saida controladas por setpoints

               % Saida do Solver. Dimensão = [ EstadosAtuais e preditos até Hp  +  DeltaU até Hc     +            Ysp                 ]
                %                             Dimensão = [               x0 = nx*(1+Hp)                          du0=nu*Hc            ysp0=ny*(1+Hp)   ]

                args.x0=[ x0;  du0;  ysp0];               % Condição incial com dados atualizados (e variáveis de decisão)

               %% Resgata estruturas de restrições guardadas pelo objeto
                args.lbx=obj.lbx;                                             % Lower Bounds para os Estados X e U do MPC
                args.ubx=obj.ubx;                                          % Upper Bounds para os Estados X e U do MPC
                args.lbg=obj.lbg;                                             % Lower Bounds para as restrições [g] que foram criadas
                args.ubg=obj.ubg;                                          % Upper Bounds para as restrições [g] que foram criadas

               %% Atualiza parâmetros que precisam ser enviados ao Solver
               % Ação atual, Setpoint das variáveis desejadas, Predição anterior e reservatório ESN entram p/ o Solver na forma de parâmetros
               %           [        Xk                  Uk;      AlvoEng;   Err_Xpredito;   Err_YPredito;       Reservatório do ModeloPreditor]
                args.p=[DadosProcesso;  Uk;       AlvoEng;      ErroX;                ErroY;                  obj.ModeloPreditor.data.a0];     
              
                %% Nova chamada do Solver
                solver=obj.casadi_solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
                Feasible=obj.casadi_solver.stats.success;
                Iteracoes=obj.casadi_solver.stats.iter_count;
%                 % Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo Hp  +   Delta U até Hc   +          Ysp          ]
                %                                   Dimensão = [            nx                      nx*(Hp)                                        nu*(Hc)                ny*(Hp+1)   ]
 
                Solucao_MPC=full(solver.x);                        % Solução ótima encontrada pelo otimizador
                x0=Solucao_MPC(1:nx*(1+Hp));                % Assume nova condição inicial para os estados X atuais e preditos 
                du0=Solucao_MPC(nx*(1+Hp)+1:nx*(1+Hp)+ nu*Hc); % Resgata novas ações de controle (Delta U) ótimos calculadas em todo o horizonte Hc
                DeltaU=du0(1:nu);                                           % Extrai DeltaU ótimo à ser aplicado
                ysp0=Solucao_MPC(nx*Hp+1+nu*Hc+1:end);    % Extrai Ysp ótimos calculados em todo o horizonte futuro
                Ysp=ysp0(1:ny);                                             % Extrai o Ysp dado pelo otimizador
                obj.contador = 0;                                            % Reinicia contador para a atuação do MPC
                TempoSolver = toc(TempoIni);                     % Tempo gasto pelo Solver
            end
            Uk=Uk+DeltaU;
            SaidaMPC=[Feasible; Iteracoes; TempoSolver; Uk; DeltaU; Ysp];
            
           % Para atualizar o modelo preditor é necessário manter a ESN atualizada
            ModeloPreditor=obj.ModeloPreditor;
            entradas_normalizadas = normaliza_entradas([Uk;DadosProcesso]);   % Normaliza entradas provenientes do processo (observar que a função nada faz com a vazão)
            [x_predito, ESNdataa0] = executa_predicao(Uk,DadosProcesso, ModeloPreditor.data.a0, ModeloPreditor, obj.EstimaVazao);
            ModeloPreditor.data.a0=ESNdataa0;     % Mantem a ESN com reservatório atualizado durante o loop de predição
            
            % Atualiza objetos da função para o próximo ciclo
            obj.x0=x0;
            obj.du0=du0;
            obj.ysp0=ysp0;
            obj.Predicao=full(x_predito);
            obj.ModeloPreditor = ModeloPreditor;
        end
    end
end

%===============================================================================
% ==================  FIM DO PROGRAMA PRINCIPAL  ================================
%===============================================================================
%% Funções para montagem das restrições
function args=Monta_lbx_ubx(args,Hp,Hc,nx,ny,dumax);

    % OBSERVAR QUE AS RESTRIÇÕES SÃO EMPILHADAS EM COLUNAS ENQUANTO SEUS
    % ARGUMENTOS (lbx/ubx, lbg/ubg) SÃO MONTADOS EM LINHA

    %% Restrições em X e U para o MPC (lbx e ubx) para o estado atual e estados futuros
    % Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo Hp  +  DeltaU até Hc  +          Ysp              ]
    %                             Dimensão =  [        nx                      nx*Hp                                            nu*Hc                    ny*(Hp+1)     ]

    % Observe que os estados X (medições) e Ysp tem restrições dinâmicas, mas sabemos que lbx/ubx não podem conter fórmulas
    % Assim, deixaremos as restrições lbx/ubx "livres" para os estados X e Ysp. As restrições serão efetivamente tratadas em g

    % Para os estados atuais e para todo o horizonte Hp
    for i=1:1+Hp                     
        args.lbx=[args.lbx, -inf(1,nx) ];          % Limites inferiores para as restrições de X      
        args.ubx=[args.ubx, inf(1,nx)];          % Limites superiores para as restrições de X
    end

    % Para as variações nas ações de controle em todo o horizonte Hc que usualmente é < Hp.
    % LEMBRAR:
    % Pelos testes que fizemos, não vale a pena usar dumin>0 e criar faixas.  Deixemos o otimizador calcular e ações pequenas
    % (mesmo que não sejam compatíveis com a resolução dos equipamentos), podem ser tratadas fora do Solver
    for i=1:Hc                                           
        args.lbx=[args.lbx, -dumax ];               % Limites inferiores para os DeltaU      
        args.ubx=[args.ubx, dumax ];              % Limites superiores para os DeltaU
    end

    % Para a Ysp atual e para todo o horizonte Hp 
    for i=1:1+Hp
        args.lbx=[args.lbx,  -inf(1,ny) ];          % Limites inferiores para os valores de Ysp      
        args.ubx=[args.ubx, inf(1,ny) ];         % Limites superiores para os valores de Ysp
    end
end
%% ==============================================================================
%% Função para extrair partes que compõe os parâmetros
function  [Xk,Uk, AlvoEng,ErroX, ErroY, Reservatorio_ESN]=ExtraiParametros(P,Indice)
    Ate=[Indice(1)   sum(Indice(1:2))  sum(Indice(1:3))  sum(Indice(1:4))  sum(Indice(1:5))  sum(Indice(1:6))];
    De=[1  Ate(1)+1   Ate(2)+1   Ate(3)+1   Ate(4)+1   Ate(5)+1  ];
    Xk=P(De(1):Ate(1));
    Uk=P(De(2):Ate(2));
    AlvoEng=P(De(3): Ate(3));
    ErroX=P(De(4):Ate(4));
    ErroY=P(De(5):Ate(5));
    Reservatorio_ESN=P(De(6):Ate(6));
end
%% ==============================================================================

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
