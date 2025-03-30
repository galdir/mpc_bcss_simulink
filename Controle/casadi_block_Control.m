%%  IMPLEMENTAÇÃO MPC USANDO O CASADI
classdef casadi_block_Control< matlab.System & matlab.system.mixin.Propagates
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

        % Matrizes de ponderação - guardadas em obj apenas para permitir o cálculo do custo na fase de execução
        Qy                                                               % Matriz para ponderar as saidas controladas por setpoint
        Qu                                                               % Matriz para ponderar as ações de controle
        R                                                                 % Matriz para ponderar as variações nas ações de controle
        Qx                                                               % Matriz para ponderar os erros de predição dos estados

        x0                                                                % Para guardar condições iniciais dos estados (X) atuais e em todo horizonte (1+Hp)
        u0                                                                 % Para guardar condições iniciais das ações de controle (U) em todo o horizonte (Hc)
        BuffDeltaFreq                                             % Para proporcionar soma de 45 últimas variações na ação de controle
        u_anterior                                                    % Armazenar o estado das variáveis manipuladas no instante anterior (Freq e PMonAlvo efetivamente lidas do processo)
        
        Predicao                                                     % Para guardar a predição no instante anterior
        ModeloPreditor                                           % Para guardar modelo de preditor e que será utilizada pelo solver para a predição
        EstimaVazao                                               % Para carregar uma única vez a 'f_Interpola_casadi_vazao_sym'
        Funcao_h                                                    % Para proceder a conta y=h(x) e obter as saidas em função da matriz h definida
        MatrizSimulador                                          % Para permitir interpolação - USADA APENAS NO IMPLEM PARA ESTIMATIVA DA PSUC*
        f_buscaLimites_sym  % para buscar limites usandos nas restricoes dinamicas

        lbx                                                                % Lower Bounds para os Estados do MPC
        ubx                                                               % Upper Bounds para os Estados do MPC
        lbg                                                                % Lower Bounds para as restrições [g] que forem criadas
        ubg                                                               % Upper Bounds para as restrições [g] que forem criadas
        umax
        umin
        dumax

        WallTime
        MargemPercentual
        contador                                                     % Variável para guardar o contador de passos de amostragem - usado para definir momentos de atuação do MPC
        NumCasasDecimais                                  % NUmero de casas decimais para limite minimo de ação do DeltaU
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
        function sz1 = getOutputSizeImpl(obj, ~)         % Organização da saida do bloco Casadi
            % Feasible (Dim=1)
            % Iteracoes (Dim=1)
            % TempoSolver (Dim=1)
            % Ação de controle (Dim=nu)               % Ação na Freq e PMonAlvo
            % DeltaU (Dim=nu)                                % DeltaU na Freq e PMonAlvo
            % SomaDeltaFreq (Dim=1)                   % Somatório da variação de fraquencia para limitar 1Hz em 7,5min
            % Jy = Custo parcial referente as saidas controladas por setpoint (Dim=1)
            % Ju = Custo parcial referente as ações de controle (Dim=1)
            % Jr = Custo parcial referente as variações nas ações de controle (Dim=1)
            % Jx = Custo parcial referente aos erros de predição das 11 variáveis (10 + vazão) (Dim=1)
            % ErroX = diferença entre as medições atuais e as predições no instante anterior (Dim = nx)
            % ErroY = diferença entre as saidas atuais e as saidas preditas no instante anterior (Dim = ny)
            % Ysp = Setpoints para as variáveis controladas por setpoint (Dim=ny)  % Setpoint para PChegada e Vazão
            % Xk = Estados atuais e futuros           % Dim = nx*(1+Hp)
            % Uk = Ações de controle para aplicar - atual e futuras (Dim = nu*Hp)

            % Carrega variáveis apenas para extrair a dimensão da saída
            Qx= evalin('base','Qx');    nx=height(Qx);  % Número de variáveis (estados) do processo
            Qu = evalin('base','Qu');  nu=height(Qu);  % Número de variáveis de entrada no processo (manipuladas)
            Qy=  evalin('base','Qy');   ny=height(Qy);  % % Número de variáveis de saida controladas por SetPoint
            Hp =  evalin('base','Hp');  obj.Hp=Hp;  % Horizonte de predição

            %        SaidaMPC=[Feasible; Iteracoes; TempoSolver; U0; DeltaU; SomaDeltaFreq;   Jy  Ju  Jr  Jx   ErroX  ErroY    Ysp;         Xk;               Uk      ];
            % dimensoes da saida do MPC

            Dim =   [1 ... % viabilidade (se viavel/feasible)
                1 ... % iteracoes
                1 ... % tempoSolver
                nu ... % U0 (manipuladas atuais)
                nu ... % DeltaU
                1 ... % SomaDeltaFreq
                1 ... % Jy (custo?)
                1 ... % Ju
                1 ... % Jr
                1 ... % Jx
                nx ... % ErroX
                ny ... % ErroY
                ny ... % Ysp
                nx*(1+Hp) ... % Xk
                nu*Hp]; % Uk

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
            obj.NumCasasDecimais=evalin('base', 'NumCasasDecimais');   % Numero de casas decimais para ações de DeltaU

            %% Funções simbólicas que serão necessárias ao Solver
            f_Interpola_casadi_vazao_sym = evalin('base', 'f_Interpola_casadi_vazao_sym');
            f_buscaLimites_sym=evalin('base', 'f_buscaLimites_sym');
            obj.f_buscaLimites_sym = f_buscaLimites_sym;

            obj.EstimaVazao=f_Interpola_casadi_vazao_sym;                  % Guarda pois a função será necessária durante a execução

            %% Carrega parâmetros definidos pelo usuário
            % Dos dados de operação
            obj.umax =  evalin('base','umax');            % Valores máximos para as entradas (Freq e PMonAlvo)
            obj.umin =  evalin('base','umin');               % Valores mínimos para as entradas (Freq e PMonAlvo)
            obj.dumax =  evalin('base','dumax');          % Variação máxima do DeltaU nas entradas (variáveis manipuladas Freq e PMonAlvo)
            obj.MargemPercentual=evalin('base','MargemPercentual');   % Margem de folga para os limites dos alarmes
            
            % Do controlador
            Hp =  evalin('base','Hp');  obj.Hp=Hp;  % Horizonte de predição
            Hc =  evalin('base','Hc');   obj.Hc=Hc; % Horizonte de controle
            PassoMPC=evalin('base','PassoMPC');    % Quantos passos de amostragem para a atuação do MPC
            obj.PassoMPC=PassoMPC;

            Qy=  evalin('base','Qy');                      % Ponderação das saidas controladas por setpoint (PChegada e Vazão)
            obj.Qy=Qy;
            Qu = evalin('base','Qu');                      % Ponderação das ações de controle nas ações de controle (Freq. e PMonAlvo)
            obj.Qu=Qu;
            R = evalin('base','R');                          % Ponderação das variações das ações de controle (Delta Freq. e Delta PMonAlvo)
            obj.R=R;
            Qx= evalin('base','Qx');                       % Ponderação para os erros de estimação das 11 variáveis do processo
            obj.Qx=Qx;

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
            UIni=evalin('base','UIni');                        % Condições iniciais das entradas (variáveis manipuladas U) do processo (em coluna)

            %% Carrega modelo preditor e esquenta a ESN
            ModeloPreditor = evalin('base', 'ModeloPreditor');   % Modelos do preditor que será usada pelo MPC
            obj.ModeloPreditor = ModeloPreditor;

            %esquenta o modelo preditor para ele estar pronto para o ponto
            %de operacao atual
            entradas_normalizadas = normaliza_entradas([UIni;XIni]);   % Normaliza entradas provenientes do processo (observar que a função nada faz com a vazão)
            for i=1:1000     % Esquenta a ESN
                % Novos estados da ESN com base no estado atual e na taxa de vazamento
                Predicao = ModeloPreditor.data.Wrr*ModeloPreditor.data.a0 +  ModeloPreditor.data.Wir*entradas_normalizadas + ModeloPreditor.data.Wbr;
                ModeloPreditor.data.a0 = (1-ModeloPreditor.data.gama)*ModeloPreditor.data.a0 + ModeloPreditor.data.gama*tanh(Predicao);
            end
            obj.ModeloPreditor = ModeloPreditor;           % Guarda como OBJ pois a ESN precisará ter seus estados internos atualizados a cada amostragem

            % gera primeira predicao para o passo de execução do mpc ter a predicao do passo anterior
            [x_predito, ~] = executa_predicao(UIni,XIni, ModeloPreditor.data.a0, ModeloPreditor, obj.EstimaVazao);
            obj.Predicao=full(x_predito);

            % Condições inciais para algumas das variáveis de decisão do MPC [  x0    u0 ]. Precisam estar em vetores coluna
            % Delta U será inicializado com zeros
            obj.x0=repmat(XIni,1+Hp,1);            % Condição incial das variáveis medidas (estados X) atuais e futuras
            obj.u0=repmat(UIni,Hp,1);                % Condição inicial para as ações de controle (U) em todo o horizonte Hp futuro
            obj.u_anterior= UIni;                          % Condição inicial das variáveis manipuladas do processo (Freq e PMonAlvo)
            
            % Inicializa com zeros o buffer que vai contabilizar o somatório das últimas variações na Frequencia
            obj.BuffDeltaFreq=zeros(45,1);

            obj.WallTime = evalin('base','WallTime'); % tempo maximo para execucao do solver

            %% =============================================================================================
            % Até aqui foi a inicialização das variáveis e estruturas, salvando em OBJ para que possam ser usadas no StepImpl

            %funcao para criar o solver
            [solver, args_solver] = cria_solver(obj.umax, obj.umin, obj.dumax, obj.MargemPercentual, ...
                obj.Hp, obj.Hc, obj.Qy, obj.Qu, obj.R, obj.Qx, obj.nx, obj.nu, obj.ny, ...
                obj.EstimaVazao, obj.f_buscaLimites_sym, obj.ModeloPreditor, obj.Funcao_h, obj.WallTime);
            obj.casadi_solver=solver;

            %% Atualizando o objeto que vai guardar os limites das restrições para oferecer na fase de implementação
            obj.lbx=args_solver.lbx;                                % Lower Bounds para as variáveis de decisão do MPC
            obj.ubx=args_solver.ubx;                             % Upper Bounds para as variáveis de decisão do MPC
            obj.lbg=args_solver.lbg;                                % Lower Bounds para as restrições [g] que foram criadas
            obj.ubg=args_solver.ubg;                             % Upper Bounds para as restrições [g] que foram criadas

        end

        %% ================  Contas de atualização -  Equivale a Flag=2 na SFunction)
        function  SaidaMPC= stepImpl(obj,X0,U0,AlvoEng,t)
%             disp(strcat("Simulação MPC em ",num2str(t)," s"));   % Só aqui usamos o tempo, útil para debug !!

            import casadi.*                                      % Importação da biblioteca Casadi (tem de estar no path)
           
            % comebntar
            DeltaUAplicado=U0-obj.u_anterior;
            obj.BuffDeltaFreq=[ DeltaUAplicado(1); obj.BuffDeltaFreq(1:end-1)];  % Atualiza buffer com últimas variações nas ações de frequencias aplicadas
            SomaDeltaFreq=sum(obj.BuffDeltaFreq);   
            SomaDeltaFreq=round(SomaDeltaFreq,obj.NumCasasDecimais);   % Não lembro a motivação para este arredondamento !!
            obj.u_anterior=U0;
            
            %% Resgata informações facilitando os cálculos para a implementação da nova ação de controle
            Funcao_h=obj.Funcao_h;    % Restaura função para fazer o cálculo das saidas:  y=h(x);

            nx=obj.nx;                 % Extrai o número de variáveis em DadosProcesso
            nu=obj.nu;                 % Extrai o número de variáveis nas Ações de controle
            ny=obj.ny;                 % Extrai o numero de variáveis de saida (controladas por setpoint)
            Hp=obj.Hp;               % Tamanho do horizonte de predição
            Hc=obj.Hc;                % Tamanho do horizonte de controle
            PassoMPC=obj.PassoMPC;   % Número de passos/amostragens para atuação do MPC

            % Inicializa custos parciais que podem compor a função objetivo
            Jy=0; Ju=0; Jr=0; Jx=0;

            XPredito=obj.Predicao;                       % Predição dos estados feita no instante anterior
            ErroX=X0-XPredito;                            % Erro entre as medições atuais e a predição feita no instante anterior

            Y0=full(Funcao_h(X0));                                      % Saidas controladas por setpoint
            YPredito=full(Funcao_h(XPredito));                  % Predição das saidas controladas por setpoint no instante anterior
            ErroY=Y0-YPredito;                             % Erro entre as medições atuais e a predição feita no instante anterior

            % Atualiza setpoint das variáveis de saida controladas por setpoint (PChegada e Vazao)
            % Considerar o AlvoEng como sendo o setpoint para as variáveis controladas por setpoint (PChegada e vazão)
            % A PChegada ótima é o próprio PMonAlvoENG
            % Vazão ótima é estimada com o FreqAlvoENG e PMonAlvoENG
            conversao_bar_kgf = 1.019716;
            Ysp= [ AlvoEng(2) ;    full(obj.EstimaVazao(AlvoEng(1),AlvoEng(2)*conversao_bar_kgf)) ];

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
                %% Para acompanhar caso de UNFEASIBLE
                if  obj.contador > PassoMPC   % (Passou por uma condição unfeasible, por isso não ressetou o contador
                    disp(strcat("Simulação MPC anterior em ",num2str(t),"s  deu unfeasible"))   % Só aqui usamos o tempo, útil para debug !!
                    disp(strcat("Esta é a tentativa ",num2str(obj.contador-PassoMPC)," para tentar achar solução ótima !!"))
                end

                TempoIni=tic;   % Inicaliza contagem de tempo para o Solver
                args=struct;     % Inicializa variável que vai armazenar a estrutura de argumentos para o solver

                %% Atualiza parâmetros que precisam ser enviados ao Solver
                %          [  Medição  Ação    AlvoEng;    Ysp     ErroX     ErroY    BuffDeltaFreq            Reservatório do ModeloPreditor]
                args.p=[    X0;          U0;     AlvoEng;    Ysp;    ErroX;     ErroY;   obj.BuffDeltaFreq;     obj.ModeloPreditor.data.a0];

                %% Condição inicial para passar ao solver (inclui variáveis de decisão)
                % Trata-se de atualização das condições inciais associadas as variáveis de decisão
                du0=zeros(Hp*nu,1);          % Inicializa valores futuros com zeros (serão variáveis de decisão tratadas por restrição de igualdade)
                args.x0_solver=[ obj.x0;  obj.u0; du0];

                %% Resgata estruturas de restrições guardadas pelo objeto
                args.lbx=obj.lbx;                                 % Lower Bounds para os Estados X e U do MPC
                args.ubx=obj.ubx;                              % Upper Bounds para os Estados X e U do MPC
                args.lbg=obj.lbg;                                 % Lower Bounds para as restrições [g] que foram criadas
                args.ubg=obj.ubg;                               % Upper Bounds para as restrições [g] que foram criadas

                %% Nova chamada do Solver
                solver=obj.casadi_solver('x0',args.x0_solver,'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
                Feasible=obj.casadi_solver.stats.success;
                Iteracoes=obj.casadi_solver.stats.iter_count;
                % Atualiza dados e aplica ação de controle se for Feasible, caso contrário, mantém tudo como está
                % Caso não seja feasible, não resseta o contador, de modo que o controlador fará nova tentativa na amostragem seguint   e
                %% Se uma solução foi encontrada
                if Feasible
                    % Saida do Solver. Dimensão = [ EstadosAtuais e futuros em todo Hp  +   U até Hp           DeltaU até Hp   ]
                    Indice = [                                                      nx*(1+Hp)                                       nu*Hp                    nu*Hp              ];
                    Solucao_MPC=full(solver.x);                          % Solução ótima encontrada pelo otimizador
                    [Xk,Uk,DeltaUk]=ExtraiSolucao(Solucao_MPC,Indice);
                    [Jy, Ju, Jr, Jx]=CalcCusto(Xk, Uk, Hp, Hc, obj.Qy, obj.Qu, obj.R, obj.Qx,ErroX,ErroY,Ysp,AlvoEng,Funcao_h);
                    % Como o solver indica novo futuro predito, atualiza x0 e u0 para os próximos ciclos
                    obj.x0=Xk;              % Guarda nova condição inicial x0 para os estados atuais e preditos pelo solver
                    obj.u0=Uk;              % Guarda ações de controle (U ótimos) atuais e preditos pelo solver
                    DeltaU=DeltaUk(1:nu);                     % Delta U como variável indicada pelo solver
                    DeltaU2=obj.u0(1:nu)-U0;                 % DeltaU = Ação ótima calculada agora, menos a ação antes aplicada
                    if norm(DeltaU-DeltaU2,2)>1e-10
                        disp(strcat("Simulação MPC em ",num2str(t)," s"))   % Só aqui usamos o tempo, útil para debug !!
                        disp('ERRO??? !!!  Checar cálculo de DeltaU - Estas contas deveriam dar resultados iguais!!!')
                    end
                    obj.contador = 0;   % Reinicia contador para a atuação do MPC
%                 else      % O solver deu unfeasible (não achou solução ótima)
%                     if t<1800 & U0<50                 % 30 min iniciais, assegura alvo para o MPA tirar da condição de unfeasible 
%                         U0=U0+[ 0.1; 0];                % Incrementa frequencia em 0.1Hz
%                     end
                end
                TempoSolver = toc(TempoIni);                          % Feasible ou não, indica tempo gasto pelo Solver
            end
            %% Independentemente do Solver ter ou não ter encontrado uma solução ótima
            DeltaU=round(DeltaU,obj.NumCasasDecimais);
            Xk=obj.x0;   % Resgata valores para disponibilizar na saida do bloco MPC
            Uk=obj.u0;   % Resgata valores para disponibilizar na saida do bloco MPC
            U0=U0+DeltaU;    % Passando ou não pelo solver, atualiza ação de controle com respectivo DeltaU
            if SomaDeltaFreq>1
                disp(strcat("Simulação MPC em ",num2str(t)," s"))   % Só aqui usamos o tempo, útil para debug !!
                disp("Não sei como poderia a soma ser > 1  !!")
            end
            if DeltaU(1)>obj.dumax(1)   % Só para debug - testar se a variação de Frequencia proposta é maior do que apermitida
                disp(strcat("Simulação MPC em ",num2str(t)," s"))   % Só aqui usamos o tempo, útil para debug !!
                disp("Não sei como poderia ter DeltaFreq > dumax  !!")
            end

            % Prepara saidas do bloco MPC
            % Dimensão = [     1             1                    1              nu     nu                   1                1    1    1   1     nx        ny         ny    nx*(1+Hp)      nu*Hp  ]
            SaidaMPC    = [Feasible; Iteracoes; TempoSolver; U0; DeltaU;  SomaDeltaFreq; Jy; Ju; Jr; Jx;  ErroX;  ErroY;  Ysp;        Xk;               Uk    ];

            % Para atualizar o modelo preditor é necessário manter o reservatório da ESN atualizado
            ModeloPreditor=obj.ModeloPreditor;        % Resgata modelo preditor
            [x_predito, ESNdataa0] = executa_predicao(U0,X0, ModeloPreditor.data.a0, ModeloPreditor, obj.EstimaVazao);
            ModeloPreditor.data.a0=ESNdataa0;      % Mantem a ESN com reservatório atualizado
            obj.ModeloPreditor = ModeloPreditor;      % Guarda modelo preditor atualizado
            obj.Predicao=full(x_predito);                     % Guarda predição para que possamos avaliar o erro de predição no próximo ciclo

        end
    end
end

%===============================================================================
% ==================  FIM DO PROGRAMA PRINCIPAL  ================================
%===============================================================================

%% ==============================================================================
%% Função para calcular os custos parciais que direcionaram o Solver
function [Jy, Ju, Jr, Jx]=CalcCusto(Xk, Uk, Hp, Hc, Qy, Qu, R, Qx,ErroX,ErroY,Ysp,AlvoEng,h)
    Jx=ErroX'*Qx*ErroX;                         % Incrementa custo do erro de predição dos estados X
    X=reshape(Xk,length(Qx),Hp+1);      % Instantes k em colunas e variáveis em linhas
    U=reshape(Uk,length(Qu),Hp);         % Instantes k em colunas e variáveis em linhas

    Jy=0;  Ju=0;  Jr=0;
    for k=1:Hp                                            % Para todo o horizonte de predição
        % Incrementa custo com a diferença entre as saidas estimadas e o setpoint desejado
        y_saida= full(h(X(:,k+1)));                             % Saida estimada (variáveis controladas por setpoint - retorna coluna)
        Jy=Jy+(y_saida-Ysp+ErroY)'*Qy*(y_saida-Ysp+ErroY);

        % Incrementa custo com a diferença entre a ação de controle e o AlvoEng, apenas até o horizonte Hc
        if k<=Hc
            Ju=Ju+(U(:,k)-AlvoEng)'*Qu*(U(:,k)-AlvoEng);
        end

        % Incrementa custo da função objetivo com o valor de DeltaU, apenas até o horizonte Hc-1
        if k<Hc
            DU=U(:,k+1)-U(:,k);
            Jr=Jr+DU'*R*DU;
        end

    end
end

%% ==============================================================================
%% Função para extrair partes que compõe a solução
function   [Xk,Uk,DeltaUk]=ExtraiSolucao(Solucao,Indice)
    Ate=[Indice(1)   sum(Indice(1:2))  sum(Indice(1:3))  ];
    De=[1  Ate(1)+1   Ate(2)+1 ];
    Xk=Solucao(De(1):Ate(1));
    Uk=Solucao(De(2):Ate(2));
    DeltaUk=Solucao(De(3):Ate(3));
end




