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

        x0                                                                % Para guardar condições iniciais dos estados (X) atuais e em todo horizonte (1+Hp) 
        u0                                                                 % Para guardar condições iniciais das ações de controle (U) em todo o horizonte (Hc)
%         ysp0                                                             % Para guardar os valores de setpoint das variáveis controladas por setpoint
        BuffDeltaFreq                                             % Para proporcionar soma de 15 variações na ação de controle

        Predicao                                                     % Para guardar a predição no instante anterior
        ModeloPreditor                                           % Para guardar modelo de preditor e que será utilizada pelo solver para a predição
        EstimaVazao                                               % Para carregar uma única vez a 'f_Interpola_casadi_vazao_sym' 
        Funcao_h                                                    % Para proceder a conta y=h(x) e obter as saidas em função da matriz h definida
        MatrizSimulador                                          % Para permitir interpolação - USADA APENAS NO IMPLEM PARA ESTIMATIVA DA PSUC*
        
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
            num = 4;                                               % São 4 pinos
            %    11 variáveis do processo (10 +  vazão)
            %     2 manipuladas (Freq e PMonAlvo aplicadas ao processo)
            %     2  = Alvos = Valores alvo ENG para as manipuladas (Freq e PMon)
            %    1 = clock, utilizado apenas para debug
            
        end
        %===============        
        function sz1 = getInputSizeImpl(~)            % Organização das entradas do bloco Casadi
            sz1=4;
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
            % AlvoOtimoCalculado (Dim=ny+1)      % Alvos Ótimos para PChegada, Vazão (saidas Y) + PSuc (apenas para compor os 3 mapas)
            % Xk = Estados atuais e futuros           % Dim = nx*(1+Hp)
            % Uk = Ações de controle para aplicar - atual e futuras (Dim = nu*Hp)

            % Carrega variáveis apenas para extrair a dimensão da saída
            Qx= evalin('base','Qx');    nx=height(Qx);  % Número de variáveis (estados) do processo
            Qu = evalin('base','Qu');  nu=height(Qu);  % Número de variáveis de entrada no processo (manipuladas)
            Qy=  evalin('base','Qy');   ny=height(Qy);  % % Número de variáveis de saida controladas por SetPoint
            Hp =  evalin('base','Hp');  obj.Hp=Hp;  % Horizonte de predição

%        SaidaMPC=[Feasible; Iteracoes; TempoSolver; U0; DeltaU; YspOut;        Xk;               Uk  ];
            Dim =          [     1             1                    1               nu     nu         ny+1       nx*(1+Hp)      nu*Hp ];
            sz1 =  sum(Dim);
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
            obj.contador = 0;                                   % Contador para indicar passos/momentos de atuação do controle MPC
                  
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
            obj.MatrizSimulador=table2array(TabelaSimulador);
            MatrizSimuladorVazao = table2array(TabelaSimulador(:,1:3));        % Especificamente as colunas Freq, PChegada e Vazao para diminuir o esforço computacional
            
            %% Carrega parâmetros definidos pelo usuário
            % Dos dados de operação
            umax =  evalin('base','umax');            % Valores máximos para as entradas (Freq e PMonAlvo)     
            umin =  evalin('base','umin');               % Valores mínimos para as entradas (Freq e PMonAlvo)     
            dumax =  evalin('base','dumax');          % Variação máxima do DeltaU nas entradas (variáveis manipuladas Freq e PMonAlvo)
            MargemPercentual=evalin('base','MargemPercentual');   % Margem de folga para os limites dos alarmes   
            LimitesMin =  evalin('base','LimitesMin');     % Limites minimos (lbx) para compor restrições
            LimitesMax =  evalin('base','LimitesMax');   % Limites máximos (ubx) para compor restrições
            
            % Do controlador
            Hp =  evalin('base','Hp');  obj.Hp=Hp;  % Horizonte de predição
            Hc =  evalin('base','Hc');   obj.Hc=Hc; % Horizonte de controle
            PassoMPC=evalin('base','PassoMPC');    % Quantos passos de amostragem para a atuação do MPC
            obj.PassoMPC=PassoMPC;
            
            Qx= evalin('base','Qx');                       % Ponderação para os erros de estimação das variáveis do processo
            Qy=  evalin('base','Qy');                      % Ponderação das saidas controladas por setpoint
            Qu = evalin('base','Qu');                      % Ponderação das ações de controle nas entradas Alvos Desejados
            R = evalin('base','R');                          % Ponderação das variações das ações de controle (Delta Freq. e Delta PMonAlvo)
         
            nx=height(Qx);   % Número de variáveis (X estados) do processo (11 = 10+ vazão)
            obj.nx=nx;
            nu=height(Qu);  % Número de variáveis de entrada no processo (U manipuladas) = Freq e PMonAlvo
            obj.nu=nu;
            ny=height(Qy);  % % Número de variáveis de saida (Y controladas por setpoint) = PChegada e Vazão
            obj.ny=ny;

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
            nx_ESN = length(ModeloPreditor.data.a0);   % Tamanho do reservatório da ESN para poder enviar o ModeloPreditor como parâmetro para o Solver
            [x_predito, ESNdataa0] = executa_predicao(UIni,XIni, ModeloPreditor.data.a0, ModeloPreditor, f_Interpola_casadi_vazao_sym);
           obj.Predicao=full(x_predito);                            % Guarda predição atual
           
           % Condições inciais para as variáveis de decisão do MPC [  x0    u0 ]. Precisam estar em vetores coluna
           obj.x0=repmat(XIni,1+Hp,1);                % Condição incial das variáveis medidas (estados X) atuais e futuras
           obj.u0=repmat(UIni,Hp,1);                % Condição inicial para as ações de controle (U) em todo o horizonte Hp futuro
%           obj.ysp0=YIni;                                         % Condição inicial para os setpoints das variáveis controladas por setpoint
           % Inicializa com zeros o buffer que vai contabilizar o somatório das últimas variações na Frequencia
           obj.BuffDeltaFreq=zeros(15,1);            
            
           %% =============================================================================================
           %% =============================================================================================
           %  Até aqui foi a inicialização das variáveis e estruturas, salvando em OBJ para que possam ser usadas no StepImpl 
           % Doravante precisamos tratar tudo de forma simbólica para o Solver
           %% =============================================================================================
           %% =============================================================================================
            %% Variáveis simbolicas para o problema de otimização
            % Para melhor entendimento, as COLUNAS representarão a evolução do instantes k das variáveis de decisão
            X =MX.sym('X',nx,1+Hp);                      % Estado atual + Estados futuros até Hp 
            U = MX.sym('U',nu,Hp);                         % Ações de controle até o horizonte Hp, mas para função custo usaremos até Hc 
            DU = MX.sym('DU',nu,Hp-1);                % Variações nas ações de controle sobre o horizonte Hp, mas para função custo usaremos até Hc-1 
            
            %% ===================================================
            % Parâmetros que foram oferecidos para o Solver
            %         [  Medições  Ações   AlvoEng;    Ysp     ErroX        ErroY    BuffDeltaFreq      Reservatório do ModeloPreditor]
            Indice=[        nx            nu           nu            ny         nx             ny            15                       nx_ESN                  ];
            % Cria vetor de parâmetros na dimensão especificada
            P=MX.sym('P',sum(Indice));
            % Associa variáveis simbólicas as respectivas partes no vetor de parâmetros
            [ Xk0,Uk0, AlvoEng,Ysp,ErroX, ErroY, BuffDeltaFreq, Reservatorio_ESN]=ExtraiParametros(P,Indice);
           
            ModeloPreditor.data.a0=Reservatorio_ESN;      % Resgata estado atual do reservatório da ESN

            %% ===================================================
            % MONTAGEM DAS RESTRIÇÕES (atuais e futuras) DAS VARIÁVEIS DE DECISÃO (lbx/ubx) 
            args=struct;     % Inicializa variável que vai armazenar a estrutura de argumentos do NLP
            args.lbx=[];       % Inicializa limites inferiores para as restrições dos estados do MPC (x0 até Hp+1, u0 até Hp)
            args.ubx=[];     % Inicializa limites superiores para as restrições dos estados do MPC
   
            % Observe que os estados X (medições) tem restrições dinâmicas, mas sabemos que lbx/ubx não podem conter fórmulas
             % Para limitar o espaço de busca do solver, estes limites podem ser definidos externamente.
             % As restrições, de fato, serão efetivamente tratadas em g

            % Para os estados atuais e para todo o horizonte Hp
            for i=1:1+Hp                     
                args.lbx=[args.lbx, LimitesMin ];       % Limites inferiores para as restrições de X   
                args.ubx=[args.ubx, LimitesMax];   % Limites superiores para as restrições de X
           end
            
            % Para as ações de controle em todo o horizonte futuro, fazemos até Hp pois o U precisará ser calculado até
            % o horizonte final. Na ponderação da função custo, porém, entra apenas até Hc
            % Para as restrições da ação U na PMonAlvo, serão também consideradas as restrições dinâmicas (em função da frequência, atual e futura)
            % da PChegada (estados X). Mas isso será devidamente tratado nas restrições em [g]
            for i=1:Hp                                           
                args.lbx=[args.lbx,    umin  ];             % Limites inferiores para a ação de controle U      
                args.ubx=[args.ubx, umax ];             % Limites superiores para a ação de controle U
            end
            
            % Restrições para os limites na variação das ações de controle
            for i=1:Hp-1                                           
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
            args.lbg=[args.lbg,   zeros(1,nu)  ];     % Restrições de igualdade para impor limite      
            args.ubg=[args.ubg, zeros(1,nu) ];   
            
            %% Restrições de igualdade para assegurar que os estados futuros vão seguir as predições
            for k=1:Hp
                % Estima passo futuro com base nos estados atuais e ações de controle atuais
                [x_predito, ESNdataa0] = executa_predicao(U(:,k),X(:,k), ModeloPreditor.data.a0, ModeloPreditor, f_Interpola_casadi_vazao_sym);
                ModeloPreditor.data.a0=ESNdataa0;     % Mantem a ESN com reservatório atualizado durante o loop de predições futuras
      
                % Restrições de igualdade para impor a dinâmica (técnica multishooting)
                g=[g;X(:,k+1)-x_predito];                          % Diferença entre os estado X futuro e o estado estimado pelo preditor
                args.lbg=[args.lbg,   zeros(1,nx)  ];          % Restrições de igualdade para forçar a dinâmica do sistema      
                args.ubg=[args.ubg, zeros(1,nx) ];   
            end
            
            %% Restrições de igualdade para definir DeltaU em função de U e tornar U como variável de decisão base  
            for k=1:Hp-1
               % Cálculo da variação na ação de controle = DeltaU
                Soma=U(:,k+1)-U(:,k)-DU(:,k);
                g=[g;Soma];                                        % Restrição de igualdade = 0 assegura DeltaU como função de U
                args.lbg=[args.lbg,   zeros(1,nu) ];    % Limite mínimo para a restrição de igualdade associada ao DeltaU      
                args.ubg=[args.ubg, zeros(1,nu)];    % Limite máximo para restrição de igualdade associada ao DeltaU
                
                % Aproveita o loop para criar restrição avaliando as variações acumuladas na frequencia
                BuffDeltaFreq=[ DU(1,k); BuffDeltaFreq(1:end-1)];    % Atualiza buffer com valor de DeltaFreq proposto
                Soma=sum(BuffDeltaFreq);
                % Avalia limites das variações 
                g=[g; Soma];                      % Insere restrição de desigualdade para o somatório do BuffDeltaFreq
                args.lbg=[args.lbg,   -1 ];  % Limite mínimo para o somatório da variação      
                args.ubg=[args.ubg,  1 ];  % Limite máximo para o somatório da variação
            end
            
            %% Restrições dinâmicas para os estados X e para as saidas Y controladas por setponit
            for k=1:Hp
%             BUSCA RESTRIÇÕES DINÂMICAS PARA SEREM TRATADAS NO LOOP
%             Lembrar que a linha 1 traz os limites máximos de todas as 11 variáveis do processo (10 + Vazão)
%             Lembrar que a linha 2 traz os limites mínimos  de todas as 11 variáveis do processo (10 + Vazão)
                LimitesX= f_buscaLimites_sym(U(1,k));  % Resgata limites (Max/Min) de alarmes para as variáveis do processo em função da frequência
                LimitesX=LimitesX';                                    % Coloca na forma de coluna
                LimitesY=h(LimitesX);                                % Extrai limites correspondentes as saidas (variáveis controladas por setpoint)

                LimitesX(:,1)=LimitesX(:,1)*(1-MargemPercentual/100);   % Implementa margem de folga em relação ao máximo
                LimitesX(:,2)=LimitesX(:,2)*(1+MargemPercentual/100);   % Implementa margem de folga em relação ao mínimo
                
                
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
                y_saida= h(X(:,k+1));                       % Saidas preditas (variáveis controladas por setpoint)

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

%            LIMITES MINIMOS
               ValMin=max(LimitesX(2,2),umin(2));   % Assume o valor mais restritivo
               DiferencaMin=U(2,k)-ValMin;              % Ação precisa ser maior do que o minimo
               g=[g; DiferencaMin];                              % Insere restrição de desigualdade, a qual precisa ser  >=0
               args.lbg=[args.lbg,    0 ];                        % Limite mínimo      
               args.ubg=[args.ubg, inf];                        % Limite máximo 

%            LIMITES MÁXIMOS
               ValMax=min(LimitesX(2,1),umax(2));   % Assume o valor mais restritivo
               DiferencaMax=ValMax-U(2,k);            % Ação precisa ser menor que o máximo
               g=[g; DiferencaMax];                              % Insere restrição de desigualdade, a qual precisa ser  >=0
               args.lbg=[args.lbg,    0 ];                        % Limite mínimo      
               args.ubg=[args.ubg, inf];                        % Limite máximo 
            end
            
            %% Preparando o custo da função objetivo
            % Lembrar que X(:,1) são as medidas atuais. Da coluna 2 em diante teremos os estados futuros estimados de 1 até Hp
            fob=0;                                                           % Inicializa custo da função objetivo
            fob=fob+ErroX'*Qx*ErroX;                         % Incrementa custo do erro de predição dos estados X
            for k=1:Hp                                                    % Para todo o horizonte de predição
                % Incrementa custo com a diferença entre as saidas estimadas e o setpoint desejado
                % Observar que o ErroY entra para zerar offset provocado pelo erro do estimador
                y_saida= h(X(:,k+1));                             % Saida estimada (variáveis controladas por setpoint - retorna coluna)
                fob=fob+(y_saida-Ysp+ErroY)'*Qy*(y_saida-Ysp+ErroY);    

                % Incrementa custo com a diferença entre a ação de controle e o AlvoEng, apenas até o horizonte Hc
                S=if_else(k>Hc,0,(U(:,k)-AlvoEng)'*Qu*(U(:,k)-AlvoEng));
                fob=fob+S;
            end
            
            for k=1:Hc-1          % Para o horizonte de controle
                % Incrementa custo da função objetivo com o valor de DeltaU, apenas até o horizonte Hc-1
                S=DU(:,k)'*R*DU(:,k);
                fob=fob+S;
            end

            %% Atualizando o objeto que vai guardar as restrições para oferecer na fase de implementação
            obj.lbx=args.lbx;                                % Lower Bounds para as variáveis de decisão do MPC
            obj.ubx=args.ubx;                             % Upper Bounds para as variáveis de decisão do MPC
            obj.lbg=args.lbg;                                % Lower Bounds para as restrições [g] que foram criadas
            obj.ubg=args.ubg;                             % Upper Bounds para as restrições [g] que foram criadas

            %% ========================Configuração do otimizador====================================
            % Monta as variáveis de decisão em um vetor coluna - esta dimensão é fundamental para entender as contas e indexações
            % Saida do Solver. Dimensão = [ EstadosAtuais + EstadosFuturos em todo HP  +        U             +     DeltaU      ]
            %                             Dimensão = [                  nx*(1+Hp)                                                    nu*Hp              nu*(Hp-1)  ]
            opt_variables=[X(:); U(:);  DU(:)];

            nlp = struct('f',fob,'x',opt_variables,'g', g, 'p', P); % Define a estrutura para problema de otimização não linear (NLP, Nonlinear Programming)
            
            %Configuração específica do IPOPT
            options=struct;
            options.print_time=0;                           % 
            options.ipopt.print_level=0;                  % [ 0 a 12] = (funciona 3 a 12) Detalhe do nivel de informação para mostrar na tela durante a execução do solver
            options.ipopt.bound_relax_factor=0;    % Tolerância absoluta para as restrições definidas pelo usuário (default=1e-8)
            
            options.ipopt.max_iter=500;              % Especifica o número máximo de iterações que o solver deve executar antes de parar.
            options.ipopt.max_wall_time=9;           % Tempo (em segundos) máximo para o solver encontrar solução
         
            solver = nlpsol('solver','ipopt', nlp,options); % Define o Interior Point OPTimizer (ipopt) para resolver o problema de otimização não linear (nlp)            
            obj.casadi_solver=solver;
            t_inicializacao = toc(tInicializa);           % Tempo gasto para a inicialização do Solver
            disp(strcat("Tempo para inicialização = ",num2str(t_inicializacao)))
     
       end
        
%% ================  Contas de atualização -  Equivale a Flag=2 na SFunction)
        function  SaidaMPC= stepImpl(obj,X0,U0,AlvoEng,t)
            disp(strcat("Simulação MPC em ",num2str(t)," s"))   % Só aqui usamos o tempo, útil para debug !!
            import casadi.*                                      % Importação da biblioteca Casadi (tem de estar no path)

            %% Resgata informações facilitando os cálculos para a implementação da nova ação de controle
            h=obj.Funcao_h;    % Restaura função para fazer o cálculo das saidas:  y=h(x);
            
            nx=obj.nx;                 % Extrai o número de variáveis em DadosProcesso
            nu=obj.nu;                 % Extrai o número de variáveis nas Ações de controle
            ny=obj.ny;                 % Extrai o numero de variáveis de saida (controladas por setpoint)
            Hp=obj.Hp;               % Tamanho do horizonte de predição
            Hc=obj.Hc;                % Tamanho do horizonte de controle
            PassoMPC=obj.PassoMPC;
            
            XPredito=obj.Predicao;                       % Predição dos estados feita no instante anterior
            ErroX=X0-XPredito;                            % Erro entre as medições atuais e a predição feita no instante anterior

            Y0=full(h(X0));                                      % Saidas controladas por setpoint        
            YPredito=full(h(XPredito));                  % Predição das saidas controladas por setpoint no instante anterior
            ErroY=Y0-YPredito;                             % Erro entre as medições atuais e a predição feita no instante anterior

            % ???????????????????????????????????????????????????????
            % Variáveis controladas por setpoint deveriam ser variáveis de decisão ?
            % ???????????????????????????????????????????????????????
            % Atualiza setpoint das variáveis de saida controladas por setpoint (PChegada e Vazao)
            % Assumimos que o setpoint ótimo para a PChegada é a PMonAlvo dado pelo solver (ação de controle)
            % Por tabela, a vazão ótima é a vazão estimada para a Freq. e PMonAlvo dadas como alvo pelo solver 
            Ysp= [ U0(2) ;   full(obj.EstimaVazao(U0(1),U0(2)*1.019716)) ];

            % Inicialização para um novo passo do Solver com base nos novos estados (entradas) medidos do processo
            % De uma forma geral, inicializar com valores atuais e toda a predição já feita antes, deve diminuir o tempo de busca do solver
            % Estes valores de x0 e u0 são sempre atualizados, passando ou não pelo solver
            obj.x0=[X0; obj.x0(1:end-nx)];  % Atualiza condição inicial dos estados com a medição atual e valores passados
            obj.u0=[U0; obj.u0(1:end-nu)];  % Atualiza condição inicial das açoes de controle entrada atual e valores passados
                     
            %% ===================== %Parâmetros e atuação do solver ========================================
            obj.contador = obj.contador+1;     % Contador ajudará a saber se é momento para atuar o controlador 
            DeltaU=zeros(nu,1);                     % Reinicia DeltaU=0, inclusive para saber quando não passou pelo Solver
            TempoSolver=0;                            % Inicializa contador para calcular o tempo gasto com o Solver, quando for o caso !!
            Feasible=0.5;                                 % Assumir padrão para indicar que não passou pelo Solver
            Iteracoes=0;                                   % Numero de iterações, alterado qdo passa pelo Solver
            %% Passo para atuação do MPC/Solver
            if (obj.contador >= PassoMPC)    % Solver só entra no passo definido pelos parâmetros de Passo do MPC ou se deu Unfeasible na amostra anteriior
                TempoIni=tic;   % Inicaliza contagem de tempo para o Solver
                args=struct;     % Inicializa variável que vai armazenar a estrutura de argumentos para o solver

                %% Atualiza parâmetros que precisam ser enviados ao Solver
               %          [  Medição  Ação    AlvoEng;    Ysp     ErroX     ErroY    BuffDeltaFreq            Reservatório do ModeloPreditor]
                args.p=[    X0;          U0;     AlvoEng;    Ysp;    ErroX;     ErroY;   obj.BuffDeltaFreq;     obj.ModeloPreditor.data.a0];     

                %% Condição inicial para passar ao solver (inclui variáveis de decisão)
                % Trata-se de atualização das condições inciais associadas as variáveis de decisão
                du0=zeros((Hp-1)*nu,1);           % Inicializa valores futuros com zeros (serão variáveis de decisão tratadas por restrição de igualdade)
                args.x0=[ obj.x0;  obj.u0; du0];                        

               %% Resgata estruturas de restrições guardadas pelo objeto
                args.lbx=obj.lbx;                                 % Lower Bounds para os Estados X e U do MPC
                args.ubx=obj.ubx;                              % Upper Bounds para os Estados X e U do MPC
                args.lbg=obj.lbg;                                 % Lower Bounds para as restrições [g] que foram criadas
                args.ubg=obj.ubg;                               % Upper Bounds para as restrições [g] que foram criadas

                %% Nova chamada do Solver
                solver=obj.casadi_solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
                Feasible=obj.casadi_solver.stats.success;
                Iteracoes=obj.casadi_solver.stats.iter_count;
                % Atualiza dados e aplica ação de controle se for Feasible, caso contrário, mantém tudo como está
                % Caso não seja feasible, não resseta o contador, de modo que o controlador fará nova tentativa na amostragem seguint   e
                %% Se uma solução foi encontrada
               if Feasible
                    % Saida do Solver. Dimensão = [ EstadosAtuais e futuros em todo Hp  +   U até Hp           DeltaU até Hp-1   ]
                    Indice = [                                                      nx*(1+Hp)                                       nu*Hp                    nu*(Hp-1)        ];
                    Solucao_MPC=full(solver.x);                          % Solução ótima encontrada pelo otimizador
                    [Xk,Uk,DeltaUk]=ExtraiSolucao(Solucao_MPC,Indice);                    
                    % Como o solver indica novo futuro predito, atualiza x0 e u0 para os próximos ciclos
                    obj.x0=Xk;              % Guarda nova condição inicial x0 para os estados atuais e preditos pelo solver
                    obj.u0=Uk;              % Guarda ações de controle (U ótimos) atuais e preditos pelo solver
                    DeltaU=DeltaUk(1:nu);                     % Delta U como variável indicada pelo solver
                    DeltaU2=obj.u0(1:nu)-U0;                 % DeltaU = Ação ótima calculada agora, menos a ação antes aplicada
                    if norm(DeltaU-DeltaU2,2)>1e-10
                        disp('ERRO??? !!!  Checar cálculo de DeltaU - Estas contas deveriam dar resultados iguais!!!')
                    end
                    obj.BuffDeltaFreq=[ DeltaU(1); obj.BuffDeltaFreq(1:end-1)];  % Atualiza buffer com últimas ações de frequencias aplicadas
%                if Feasible
                   obj.contador = 0;                                            % Reinicia contador para a atuação do MPC
               end
                TempoSolver = toc(TempoIni);                          % Tempo gasto pelo Solver
            end
            %% Independentemente do Solver ter encontrado uma solução ótima
            U0=U0+DeltaU;    % Passando ou não pelo solver, atualiza ação de controle com respectivo DeltaU

            % Com base nos resultados do solver, atualiza setpoint das variáveis de saida controladas por setpoint (PChegada e Vazao)
            Ysp = [ U0(2) ;   full(obj.EstimaVazao(U0(1),U0(2)*1.019716)) ];
            % Setpoint para a PSuc não existe. Usamos a interpolação para oferecer número coerente com a Freq. e PChegada ditas ótimas
            PSucOtima=Interpola(U0(1),U0(2)*1.019716,obj.MatrizSimulador,8);
            YspOut=[Ysp; PSucOtima];
            Xk=obj.x0;
            Uk=obj.u0;
%         Dimensão = [     1             1                    1              nu     nu           ny+1    nx*(1+Hp)      nu*Hp ]
            SaidaMPC=[Feasible; Iteracoes; TempoSolver; U0; DeltaU; YspOut;        Xk;               Uk   ];
            
           % Para atualizar o modelo preditor é necessário manter o reservatório da ESN atualizado 
            ModeloPreditor=obj.ModeloPreditor;        % Resgata modelo preditor
            [x_predito, ESNdataa0] = executa_predicao(U0,X0, ModeloPreditor.data.a0, ModeloPreditor, obj.EstimaVazao);
            ModeloPreditor.data.a0=ESNdataa0;      % Mantem a ESN com reservatório atualizado
            obj.ModeloPreditor = ModeloPreditor;      % Guarda modelo preditor atualizado
            obj.Predicao=full(x_predito);                     % Guarda predição para que possamos avaliar o erro de predição no próximo ciclo
            
            %% Para acompanhar caso de UNFEASIBLE 
            if  obj.contador > PassoMPC   % (Passou por uma condição unfeasible, por isso não ressetou o contador
                disp("Ao menos uma restrição foi violada. Nova tentativa do MPC será feita já na próxima amostragem.") 
                disp("Este momento merece um bom ALARME, pois, ser persistir, o TRIP invitavelmente vai acontecer")
                disp("Se a folga dada em MargePercentual for ZERO,  o TRIP está muito próximo e não sei se há tempo para outras ações") 
                disp("A MargemPercentual refletirá no tempo de folga para alguma ação antes do TRIP")
                disp("Desabilitar o MPC? Mas quem assume?")
            end
            if  obj.contador >9   % (9 passos de 10s = 1,5min)
                disp("Não conseguimos evitar restrições fossem violadas ... o TRIP pode acontecer!!") 
                disp("Se a folga dada em MargePercentual for ZERO, o TRIP vai acontecer, se já não aconteceu") 
                disp("Se há folga dada em MargemPercentual o TRIP pode acontecer a qualquer momento e o MPC está inerte")
            end
        end
    end
end

%===============================================================================
% ==================  FIM DO PROGRAMA PRINCIPAL  ================================
%===============================================================================

%% ==============================================================================
%% Função para extrair partes que compõe os parâmetros do Solver
function   [ Xk0,Uk0,AlvoEng,Ysp,ErroX, ErroY, BuffDeltaFreq, Reservatorio_ESN]=ExtraiParametros(P,Indice);
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
%% ==============================================================================
%% Função para extrair partes que compõe a solução
function   [Xk,Uk,DeltaUk]=ExtraiSolucao(Solucao,Indice);
    Ate=[Indice(1)   sum(Indice(1:2))  sum(Indice(1:3))  ];
    De=[1  Ate(1)+1   Ate(2)+1 ];
    Xk=Solucao(De(1):Ate(1));
    Uk=Solucao(De(2):Ate(2));
    DeltaUk=Solucao(De(3):Ate(3));
end
%% ==============================================================================

%% ==================        Outras funções adicionais   ================================
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
