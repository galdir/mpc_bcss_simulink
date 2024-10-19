%%  IMPLEMENTAÇÃO MPC USANDO A BIBOIOTECA CASADI
classdef casadi_block_Control < matlab.System & matlab.system.mixin.Propagates
    properties (DiscreteState)
    end
    properties (Access = private)                       % Criação das variáveis que vão compor o OBJETO
        casadi_solver                                            % Criação do solver Casadi
        
        % Variáveis que representarão a condição inicial para o solver em cada operação do Solver
        Hp                                                               % Horizonte de predição
        Hc                                                                % Horizonte de controle
        nx                                                                 % Numero de variáveis de entrada (estados) do processo
        ny                                                                 % Numero de variáveis (saida do processo) que são controladas por setpoint
        nu                                                                 % Numero de variáveis manipuladas (ações de controle possíves) - no caso, Freq e PMonAlvo
        PassoMPC                                                  % Numero de amostragens até a atuação do MPC

        x0                                                                % Para guardar as variáveis medidas (estados X) atuais e em todo horizonte (1+Hp) 
        du0                                                              % Para guardar as ações de controle (DeltaU) em todo o horizonte (Hc)
        ysp0                                                            % Para guardar os setpoints ótimos calculados para as variáveis controladas por setpoint
        BuffDeltaFreq                                             % Para proporcionar soma de 15 variações na ação de controle

        Predicao                                                     % Para guardar a predição no instante anterior
        ModeloPreditor                                           % Para guardar modelo de preditor e que será utilizada pelo solver para a predição
        EstimaVazao                                               % Para carregar uma única vez a 'f_Interpola_casadi_vazao_sym' 
        Funcao_h                                                    % Para proceder a conta y=h(x) e obter as saidas em função da matriz h definida
        
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
            Hp =  evalin('base','Hp');  obj.Hp=Hp;  % Horizonte de predição
            Hc =  evalin('base','Hc');   obj.Hc=Hc; % Horizonte de controle
            PassoMPC=evalin('base','PassoMPC');    % Quantos passos de amostragem para a atuação do MPC
            obj.PassoMPC=PassoMPC;
            
            Qx= evalin('base','Qx');                       % Ponderação para os erros de estimação das variáveis do processo
% %            Peso=normaliza_entradas(ones(12,1)); % Extrai fator usado para normalização
% %            Peso=[ Peso(3:end); 1/500];                           % Cria fator de correção para a ponderação das variáveis
%             % Pesos para ponderação
%             %             PSuc  PChegada    PDiff    PDescarga   Tmotor  ITorque    ITotal     TSuc  Vibração   TChegada  Vazao
%             Peso = [  1/100      1/50         1/100       1/100          1/150    1/150       1/180     1/100       1/3         1/100        1/500];     
%             Peso=diag(Peso);
%             Qx=Qx*Peso;                                            % Ajusta matriz de pesos em função das grandezas
           
            Qu = evalin('base','Qu');                      % Ponderação das ações de controle nas entradas Alvos Desejados
            R = evalin('base','R');                          % Ponderação das variações das ações de controle (Delta Freq. e Delta PMonAlvo)
% %             %             Freq.     PMonAlvo
%             Peso = [  1/60          1/50     ];     
%             Peso=diag(Peso);
%             Qu=Qu*Peso;                                            % Ajusta matriz de pesos em função das grandezas
%             R=R*Peso;                                                 % Ajusta matriz de pesos em função das grandezas
            
            Qy=  evalin('base','Qy');                      % Ponderação das saidas controladas por setpoint
% %             %             PChegada     Vazao
%             Peso = [  1/50               1/500  ];     
%             Peso=diag(Peso);
%             Qy=Qy*Peso;                                            % Ajusta matriz de pesos em função das grandezas
         
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
            XIni=evalin('base','XIni');                        % Condições iniciais das variáveis (estados) do processo (em coluna)
            YIni = full(h(XIni));                                   % Valor das variáveis de saida do processo (Y controladas por setpoint) na forma de coluna
            UIni=evalin('base','UIni');                        % Condições iniciais das entradas (variáveis manipuladas U) do processo (em coluna)
                     
           %% Carrega modelo preditor e esquenta a ESN
            ModeloPreditor = evalin('base', 'ModeloPreditor');   % Modelos do preditor que será usada pelo MPC
%        TipoPreditor=ModeloPreditor.data.tipo;                    % Verifica tipo 1=ESN, 2 = LSTM (USAMOS APENAS ESN)
            entradas_normalizadas = normaliza_entradas([UIni;XIni]);   % Normaliza entradas provenientes do processo (observar que a função nada faz com a vazão)
            for i=1:1000     % Esquenta a ESN
               % Novos estados da ESN com base no estado atual e na taxa de vazamento
                Predicao = ModeloPreditor.data.Wrr*ModeloPreditor.data.a0 +  ModeloPreditor.data.Wir*entradas_normalizadas + ModeloPreditor.data.Wbr; 
                ModeloPreditor.data.a0 = (1-ModeloPreditor.data.gama)*ModeloPreditor.data.a0 + ModeloPreditor.data.gama*tanh(Predicao);
            end
            obj.ModeloPreditor = ModeloPreditor;           % Guarda como OBJ pois a ESN precisará ter seus estados internos atualizados a cada amostragem
            nx_ESN = length(ModeloPreditor.data.a0);  % Tamanho do reservatório da ESN para poder enviar o ModeloPreditor como parâmetro para o Solver
            [x_predito, ESNdataa0] = executa_predicao(UIni,XIni, ModeloPreditor.data.a0, ModeloPreditor, f_Interpola_casadi_vazao_sym);
           obj.Predicao=full(x_predito);                            % Guarda predição atual
           
           % Condições inciais para as variáveis de decisão do MPC [  x0   du0    Ysp]
           obj.x0 = repmat(XIni,1+Hp,1);               % Condição das variáveis medidas (estados X) atuais e futuras
           obj.ysp0=YIni;                                          % Inicializa Ysp com os valores atuais na saida da planta
           du0=zeros(Hc,nu);                                 % Estrutura para armazenar DeltaU atuais e futuros, até o horizonte Hc 
           obj.du0=du0(:);                                       % Condição inicial = zero para os DeltaU em todo o horizonte Hc futuro
           obj.BuffDeltaFreq=zeros(15,1);            % Inicializa com zeros o buffer que vai contabilizar o somatório das últimas variações na Frequencia
            
           %% =============================================================================================
           %  Até aqui foi a inicialização das variáveis e estruturas, salvando em OBJ para que possam ser usadas no StepImpl 
           % Doravante precisamos tratar tudo de forma simbólica para o Solver
           %% =============================================================================================
            %% Variáveis simbolicas para o problema de otimização
            % Para melhor entendimento, as linhas representarão a evolução do instantes k
            X =MX.sym('X',1+Hp,nx);                      % Estado atual + Estados futuros até Hp 
            du = MX.sym('du',Hc,nu);                      % Incrementos do controle sobre o horizonte Hc (Variável de decisão)
            DU = [du;zeros((Hp-Hc),nu)];                % Sequencia de ação de controle com peso zero para horizonte maior que Hc
%             DU = MX.sym('DU',Hc,nu);                      % Incrementos do controle sobre o horizonte Hc (Variável de decisão)
            U = MX.sym('DU',Hc,nu);                      % Ações de controle sobre o horizonte Hc (temporário pois não é a variável de decisão)

            Ysp = MX.sym('Ysp',1,ny);                     % Set-point otimo calculado pelo otimizador (Variável de decisão)
            
            % Parâmetros que devem ser oferecidos para o Solver
            %         EntradaAtual   Ação atual   DeltaU    AlvoENG    ErroX     ErroY   Buffer   Reservatório da ESN
            Indice=[  nx                        nu            nu*Hc          nu            nx             ny         15       nx_ESN];
            % Cria vetor de parâmetros na dimensão especificada
            P =MX.sym('P',sum(Indice));
            % Associa variáveis simbólicas as respectivas partes no vetor de parâmetros
            [ Xk0,Uk0, DU0,AlvoEng,ErroX, ErroY, BuffDeltaFreq, Reservatorio_ESN]=ExtraiParametros(P,Indice);
            % Coloca em formato de linha para padronizar as contas que virão
            Xk0=Xk0';   Uk0=Uk0';  DU0=reshape(DU0,Hc,nu);   AlvoEng=AlvoEng';  ErroX=ErroX';  ErroY=ErroY';        
            % NÃO USEI DU0 - LIMPAR ISSO ??? !!!!   Pensei em usar para favorecer a condição inicial dos DeltaU em todo o horizonte
           
            ModeloPreditor.data.a0=Reservatorio_ESN;      % Resgata estado atual do reservatório da ESN
            %% ===================================================
            % Montagem das restrições (atuais e futuras) dos estados X (lbx/ubx) e
            % restrições "livres" que podem ser de igualdade ou desigualdade (lbg/ubg)
            args=struct;     % Inicializa variável que vai armazenar a estrutura de argumentos do NLP
            args.lbx=[];       % Inicializa limites inferiores para as restrições dos estados do MPC (x0 até Hp+1, du0 até Hc  Ysp0)
            args.ubx=[];     % Inicializa limites superiores para as restrições dos estados do MPC
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
            fob=fob+ErroX*Qx*ErroX';                         % Incrementa custo do erro de predição dos estados X

            % Multiple-shooting = restrições de igualdade para seguir dinâmica do sistema
            g=[g; X(1,:)'-Xk0'];                                 % Diferença entre os estados X e as medições do sistema
            args.lbg=[args.lbg,   zeros(1,nx)  ];     % Restrições de igualdade para impor a dinâmica do sistema      
            args.ubg=[args.ubg, zeros(1,nx) ];   
            
%             X(1,:)=Xk0;                                           % Assume entradas atuais do processo
            % Para todo o horizonte de predição futuro
            for k=1:Hp
                [x_predito, ESNdataa0] = executa_predicao(Uk0',X(k,:)', ModeloPreditor.data.a0, ModeloPreditor, f_Interpola_casadi_vazao_sym);
                ModeloPreditor.data.a0=ESNdataa0;     % Mantem a ESN com reservatório atualizado durante o loop de predição
                y_predito= h(x_predito);                           % Saida (variáveis controladas por setpoint)
                fob=fob+(y_predito'-Ysp+ErroY)*Qy*(y_predito'-Ysp+ErroY)';   % Incrementa função objetivo com erro entre as predições (atuais e futuras) 
               % O Ysp é uma variável de decisão e o erro de estimador da saida (ErroY) é único em todo
               % o horizonte de predição. Tudo isso ponderado pela matriz Qy
               
                % Restrições de igualdade para impor a dinâmica
                g=[g;X(k+1,:)'-x_predito];                          % Diferença entre os estados X e as medições do sistema
                args.lbg=[args.lbg,   zeros(1,nx)  ];          % Restrições de igualdade para forçar a dinâmica do sistema      
                args.ubg=[args.ubg, zeros(1,nx) ];   

                % Atualiza ação de controle com DeltaU 
                Uk0=Uk0+ DU(k,:);
                BuffDeltaFreq=ShiftDown(BuffDeltaFreq,DU(k,1));    % Atualiza buffer com valor de DeltaFreq proposto
                g=[g; sum(BuffDeltaFreq)];         % Insere restrição de desigualdade para o somatório do BuffDeltaFreq
                args.lbg=[args.lbg,   -1 ];  % Limite mínimo para o somatório da variação      
                args.ubg=[args.ubg,  1 ];  % Limite máximo para o somatório da variação

               % Para o somatório até Hc
                S=if_else(k>Hc,0,(Uk0-AlvoEng)*Qu*(Uk0-AlvoEng)');
                fob=fob+S;

                % Para o somatório até Hc-1
                S=if_else(k>Hc-1,0,DU(k,:)*R*DU(k,:)');
                fob=fob+S;

                % APROVEITA O LOOP PARA CRIAR DEMAIS RESTRIÇÕES [g] PARA TODO O HORIZONTE Hp

                % Restrições e custo da função objetivo na ação de controle (Freq e PMonAlvo)
                g=[g; DU(k,:)'];                            % Insere restrição de desigualdade para o DeltaU
                args.lbg=[args.lbg,   -dumax ];  % Limite mínimo para a restrição de desigualdade associada ao DeltaU      
                args.ubg=[args.ubg,  dumax ];  % Limite máximo para restrição de desigualdade associada ao DeltaU

                % Restrições para a Frequencia
                g=[g; Uk0(1)];                                 % Insere restrição de desigualdade especificamente para a Frequencia
                args.lbg=[args.lbg,    umin(1) ];    % Limite mínimo para a frequência      
                args.ubg=[args.ubg,  umax(1) ];   % Limite máximo para a frequência

                % CALCULA RESTRIÇÕES DINÂMICAS PARA SEREM TRATADAS NO LOOP
                LimitesX= f_buscaLimites_sym(Uk0(1));  % Resgata limites de alarmes para as variáveis do processo em função da frequência
                LimitesY=matriz_h*LimitesX';                     % Extrai limites correspondentes as saidas (variáveis controladas por setpoint)

                LimitesX(1,:)=LimitesX(1,:)*(1-MargemPercentual/100);   % Implementa margem de folga em relação ao máximo
                LimitesX(2,:)=LimitesX(2,:)*(1+MargemPercentual/100);   % Implementa margem de folga em relação ao mínimo
                
                % RESTRIÇÕES PARA AS VARIÁVEIS DO PROCESSO (ESTADOS X)
                % Insere restrições para os valores máximos das variáveis (estados X) preditos
                LimMaxX=LimitesX(1,:)'-x_predito;        % Para não ser violado o limite, a diferença deve ser >= 0
                g=[g; LimMaxX]; 
                args.lbg=[args.lbg,  zeros(1,nx) ];     % Limite mínimo para restrição de desigualdade      
                args.ubg=[args.ubg, inf(1,nx)];          % Limite máximo para restrição de desigualdade
                % Insere restrições para os valores mínimos das variáveis (estados X) preditos
                LimMinX=x_predito-LimitesX(2,:)';    % Para não ser violado o limite, a diferença deve ser >= 0
                g=[g; LimMinX]; 
                args.lbg=[args.lbg,   zeros(1,nx) ];    % Limite mínimo para restrição de desigualdade      
                args.ubg=[args.ubg,  inf(1,nx) ];       % Limite máximo para restrição de desigualdade
                
                % RESTRIÇÕES PARA AS VARIÁVEIS DE SAIDA (CONTROLADAS POR SETPOINT)
                % Insere restrições para os valores máximos das saidas controladas por setpoint que são preditas
                LimMaxY=LimitesY(1,:)'-y_predito;    % Para não ser violado o limite, a diferença deve ser >= 0
                g=[g; LimMaxY]; 
                args.lbg=[args.lbg,  zeros(1,ny) ];      % Limite mínimo para restrição de desigualdade      
                args.ubg=[args.ubg, inf(1,ny) ];          % Limite máximo para restrição de desigualdade
                
                % Insere restrições para os valores mínimos saidas controladas por setpoint que são preditas
                LimMinY=y_predito-LimitesY(2,:)';      % Para não ser violado o limite, a diferença deve ser >= 0
                g=[g; LimMinY]; 
                args.lbg=[args.lbg, zeros(1,ny) ];         % Limite mínimo para restrição de desigualdade      
                args.ubg=[args.ubg,inf(1,ny) ];             % Limite máximo para restrição de desigualdade
                
               % Para as restrições da ação U na PMonAlvo, serão também consideradas as restrições dinâmicas da PChegada (estados X)
                LimiteMinPmonAlvo=max(LimitesX(2,2),umin(2));   % Assume a condição do limite minimo mais restritivo
                LimiteMaxPmonAlvo=min(LimitesX(1,2),umax(2));  % Assume a condição do limite máximo mais restritivo
                g=[g; Uk0(2)];                                                   % Insere restrição de desigualdade especificamente na PMonAlvo
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
%             du=DU(1:Hc,:);
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
        function  SaidaMPC= stepImpl(obj,DadosProcesso,U0,AlvoEng)
%             disp(strcat("Simulação MPC em ",num2str(t)," s"))
            import casadi.*                                      % Importação da biblioteca Casadi (tem de estar no path)

            %% Resgata informações facilitando os cálculos para a implementação da nova ação de controle
            h=obj.Funcao_h;    % Restaura função para fazer o cálculo das saidas:  y=h(x);
            
            nx=obj.nx;                 % Extrai o número de variáveis em DadosProcesso
            nu=obj.nu;                 % Extrai o número de variáveis nas Ações de controle
            ny=obj.ny;                 % Extrai o numero de variáveis de saida (controladas por setpoint)
            Hp=obj.Hp;               % Tamanho do horizonte de predição
            Hc=obj.Hc;                % Tamanho do horizonte de controle
            PassoMPC=obj.PassoMPC;
            
            XPredito=obj.Predicao;                         % Predição dos estados feita no instante anterior
            ErroX=DadosProcesso-XPredito;       % Erro entre as medições atuais e a predição feita no instante anterior

            SaidasProcesso=full(h(DadosProcesso)); % Saidas controladas por setpoint        
            YPredito=full(h(XPredito));       % Predição das saidas controladas por setpoint no instante anterior
            ErroY=SaidasProcesso-YPredito;       % Erro entre as medições atuais e a predição feita no instante anterior
         
            % Inicialização para um novo passo do Solver com base nos novos estados (entradas) medidos do processo
            % De uma forma geral, inicializar com valores atuais e toda a predição já feita antes, deve diminuir o tempo de busca do solver
            obj.x0=ShiftDown(obj.x0,DadosProcesso);  % Atualiza condição inicial dos estados com a medição atual e valores passados
                     
            %% ===================== %Parâmetros e atuação do solver ========================================
            obj.contador = obj.contador+1;     % Contador ajudará a saber se é momento para atuar o controlador 
            DeltaU=zeros(nu,1);                     % Reinicia DeltaU=0, inclusive para saber quando não passou pelo Solver
            TempoSolver=0;                            % Inicializa contador para calcular o tempo gasto com o Solver, quando for o caso !!
            Feasible=0.5;                                 % Assumir padrão para indicar que não passou pelo Solver
            Iteracoes=0;                                   % Numero de iterações, alterado qdo passa pelo Solver
            %% Passo para atuação do MPC/Solver
            if (obj.contador==PassoMPC)      % Solver só entra no passo definido pelos parâmetros de Passo do MPC
                TempoIni=tic;   % Inicaliza contagem de tempo para o Solver
                args=struct;     % Inicializa variável que vai armazenar a estrutura de argumentos para o solver

                %% Atualiza parâmetros que precisam ser enviados ao Solver
               %           [        Medidas       U0;       du0         AlvoEng;   Err_Xpredito;   Err_YPredito;      BuffDeltaFreq        Reservatório do ModeloPreditor]
                args.p=[DadosProcesso;  U0;    obj.du0;     AlvoEng;      ErroX;               ErroY;           obj.BuffDeltaFreq;     obj.ModeloPreditor.data.a0];     

                %% Condição inicial para passar ao solver (inclui variáveis de decisão)
                % Atualiza setpoint das variáveis de saida em função dos novos Alvos da Engenharia
                ysp0 = [ AlvoEng(2) ;   full(obj.EstimaVazao(AlvoEng(1),AlvoEng(2))) ];      % Setpoint para PChegada é a PMonAlvoENG e para Vazao é estimada em função da FreqAlvo e PMonAlvo
                
                % Atualiza condição inicial dos deltaU atual (zeros) e valores antes aplicados
%                du0=ShiftDown(obj.du0,zeros(1,nu));  
                du0=obj.du0;       % Inicializa delta U com o hostórico do DeltaU aplicado  

                % Saida do Solver. Dimensão = [ EstadosAtuais e preditos até Hp  +  DeltaU até Hc     +            Ysp       ]
                %                             Dimensão = [               x0 = nx*(1+Hp)                       du0=nu*Hc                 ysp0=ny    ]
                args.x0=[ obj.x0;  du0;  ysp0];               % Atualizada condição incial das variáveis de decisão com base nos valores antes praticados

               %% Resgata estruturas de restrições guardadas pelo objeto
                args.lbx=obj.lbx;                                             % Lower Bounds para os Estados X e U do MPC
                args.ubx=obj.ubx;                                          % Upper Bounds para os Estados X e U do MPC
                args.lbg=obj.lbg;                                             % Lower Bounds para as restrições [g] que foram criadas
                args.ubg=obj.ubg;                                          % Upper Bounds para as restrições [g] que foram criadas

                %% Nova chamada do Solver
                solver=obj.casadi_solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
                Feasible=obj.casadi_solver.stats.success;
                Iteracoes=obj.casadi_solver.stats.iter_count;
                
                % Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo Hp  +   Delta U até Hc   +      Ysp  ]
                %                                   Dimensão = [            nx                      nx*(Hp)                                   nu*(Hc)                ny   ]
 
                Solucao_MPC=full(solver.x);                          % Solução ótima encontrada pelo otimizador
                % Atualiza objetos da função para o próximo ciclo (x0 deve ser atualizado fora do loop pois nem sempre passará pelo Solver)
                obj.x0=Solucao_MPC(1:nx*(1+Hp));             % Assume nova condição inicial x0 para os estados atuais e preditos 
                obj.du0=Solucao_MPC(nx*(1+Hp)+1:nx*(1+Hp)+nu*Hc); % Resgata novas ações de controle (Delta U) ótimos calculadas em todo o horizonte Hc
                DeltaU=obj.du0(nu,1);                                     % Extrai DeltaU ótimo à ser aplicado (primeiro da fila)
                obj.ysp0=Solucao_MPC(end-ny+1:end);       % Extrai Ysp ótimos calculados (últimas ny colunas do vetor)
                obj.BuffDeltaFreq=ShiftDown(obj.BuffDeltaFreq,DeltaU);

                obj.contador = 0;                                            % Reinicia contador para a atuação do MPC
                TempoSolver = toc(TempoIni);                     % Tempo gasto pelo Solver
            end
            U0=U0+DeltaU;    % Passando ou não pelo solver, atualiza ação de controle com respectivo DeltaU
%             Ysp0=obj.ysp0;
            SaidaMPC=[Feasible; Iteracoes; TempoSolver; U0; DeltaU; obj.ysp0];
            
           % Para atualizar o modelo preditor é necessário manter o reservatório da ESN atualizado 
            ModeloPreditor=obj.ModeloPreditor;   % Resgata modelo preditor
            entradas_normalizadas = normaliza_entradas([U0;DadosProcesso]);   % Normaliza entradas provenientes do processo (observar que a função nada faz com a vazão)
            [x_predito, ESNdataa0] = executa_predicao(U0,DadosProcesso, ModeloPreditor.data.a0, ModeloPreditor, obj.EstimaVazao);
            ModeloPreditor.data.a0=ESNdataa0;      % Mantem a ESN com reservatório atualizado
            obj.ModeloPreditor = ModeloPreditor;      % Guarda modelo preditor atualizado
            obj.Predicao=full(x_predito);                     % Guarda predição para que possamos avaliar o erro de predição no próximo ciclo
            
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
    % Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo Hp  +  DeltaU até Hc      +      Ysp    ]
    %                             Dimensão =  [        nx                      nx*Hp                                            nu*Hc                        ny    ]

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

    % Para a Ysp
    args.lbx=[args.lbx,  -inf(1,ny) ];          % Limites inferiores para os valores de Ysp      
    args.ubx=[args.ubx, inf(1,ny) ];         % Limites superiores para os valores de Ysp
end
%% ==============================================================================
%% Função para extrair partes que compõe os parâmetros
function   [ Xk0,Uk0, DU0,AlvoEng,ErroX, ErroY, BuffDeltaFreq, Reservatorio_ESN]=ExtraiParametros(P,Indice);
    Ate=[Indice(1)   sum(Indice(1:2))  sum(Indice(1:3))  sum(Indice(1:4))  sum(Indice(1:5))  sum(Indice(1:6))  sum(Indice(1:7))  sum(Indice(1:8))  ];
    De=[1  Ate(1)+1   Ate(2)+1   Ate(3)+1   Ate(4)+1   Ate(5)+1   Ate(6)+1    Ate(7)+1];
    Xk0=P(De(1):Ate(1));
    Uk0=P(De(2):Ate(2));
    DU0=P(De(3):Ate(3));
    AlvoEng=P(De(4): Ate(4));
    ErroX=P(De(5):Ate(5));
    ErroY=P(De(6):Ate(6));
    BuffDeltaFreq=P(De(7):Ate(7));
    Reservatorio_ESN=P(De(8):Ate(8));
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
