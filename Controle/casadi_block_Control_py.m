%%  IMPLEMENTAÇÃO MPC USANDO O CASADI
%atual
classdef casadi_block_Control_py< matlab.System & matlab.system.mixin.Propagates
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
        arquivo_saida_ddmpc
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
            obj.arquivo_saida_ddmpc = 'C:\Users\galdir\Documents\GitHub\ddmpc_bcss\variaveis_saida_extendida.csv';
            deletar_arquivo_saidas_ddmpc(obj.arquivo_saida_ddmpc)
            
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
            obj.x0=repmat(XIni,1+Hp,1);                % Condição incial das variáveis medidas (estados X) atuais e futuras
            obj.u0=repmat(UIni,Hp,1);                % Condição inicial para as ações de controle (U) em todo o horizonte Hp futuro

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
            disp(strcat("Simulação MPC em ",num2str(t)," s"))   % Só aqui usamos o tempo, útil para debug !!
            %escrever arquivo para o ddmpc python
            arquivo_entrada_ddmpc = 'C:\Users\galdir\Documents\GitHub\ddmpc_bcss\variaveis_entrada.csv';
            escreve_entradas_ddmpc(X0, U0, AlvoEng, arquivo_entrada_ddmpc);
            

            % Inicializa custos parciais que podem compor a função objetivo
            Jy=0; Ju=0; Jr=0; Jx=0;
            DeltaU=zeros(1, obj.nu);                     % Reinicia DeltaU=0, inclusive para saber quando não passou pelo Solver
            TempoSolver=0;                            % Inicializa contador para calcular o tempo gasto com o Solver, quando for o caso !!
            Feasible=0.5;                                 % Assumir padrão para indicar que não passou pelo Solver
            Iteracoes=0;                                   % Numero de iterações, alterado qdo passa pelo Solver
            SomaDeltaFreq = 0;
            ErroX = zeros(1, obj.nx);  
            ErroY = zeros(1, obj.ny);
            Ysp = zeros(1, obj.ny);
            Xk = zeros(1, obj.nx * (obj.Hp+1));
            Uk = zeros(1, obj.nu * obj.Hp);
            
            %pause(5);
            saidas = ler_saidas_ddmpc_quando_modificado(obj.arquivo_saida_ddmpc);
            if size(saidas) > 0
                Feasible = saidas.feasible;
                Iteracoes = saidas.iteracoes;
                TempoSolver = saidas.tempo_solver;
                DeltaU = [saidas.du0_0, saidas.du0_1];
                SomaDeltaFreq = saidas.soma_delta_freq;
                Jy=saidas.jy; 
                Ju=saidas.ju; 
                Jr=saidas.jr; 
                Jx=saidas.jx;
                ErroX = saidas.erro_x;
                ErroY = saidas.erro_y;
                Ysp = [saidas.ysp_0, saidas.ysp_1];
                Xk = saidas.xk;
                Uk = saidas.uk;
                if(saidas.timestamp~=-1)
                    U0 = [saidas.u0_0, saidas.u0_1]';
                end
            end
                

            % Prepara saidas do bloco MPC
            % Dimensão = [     1             1                    1              nu     nu                   1                1    1    1   1     nx        ny         ny    nx*(1+Hp)      nu*Hp  ]
            SaidaMPC    = [Feasible; Iteracoes; TempoSolver; U0; DeltaU';  SomaDeltaFreq; Jy; Ju; Jr; Jx;  ErroX';  ErroY';  Ysp';        Xk';               Uk'    ];

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

function escreve_entradas_ddmpc(X0, U0, AlvoEng, nome_arquivo)

    % Define as chaves e seus valores de tags
    tags = struct();
    tags.pressao_succao_BCSS = 'M54PI103E/1.PV';
    tags.pressao_chegada = 'T61PSI033/1.PV';
    tags.pressao_diferencial_BCSS = 'M54PDI109E/1.PV';
    tags.pressao_descarga_BCSS = 'M54PI104E/1.PV';
    tags.temperatura_motor_BCSS = 'M54TI106E/1.PV';
    tags.corrente_torque_BCSS = 'M54II108E/1.PV';
    tags.corrente_total_BCSS = 'M54IQI117E/1.PV';
    tags.temperatura_succao_BCSS = 'M54TI105E/1.PV';
    tags.vibracao_BCSS = 'M54VXI107E/1.PV';
    tags.temperatura_chegada = 'T61TI035/1.PV';
    
    tags.frequencia_BCSS = 'M54SI111E/1.PV_OS';
    tags.pressao_montante_alvo = 'TE4104_PMON_ALVO';
    
    tags.freq_alvo_eng = 'TE4104_FREQ_ALVO';
    %tags.freq_BCSS_entrada_VSD = '301072_M54_SI_112_E';
    tags.pressao_montante_alvo_eng = 'pressao_montante_alvo_eng';

    % Assume que seus dados estão em um array chamado 'dados'
    % dados = [valor1 valor2 valor3 ...]; % seu array de dados aqui
    dados = [X0(1:end-1);U0;AlvoEng]; %removendo vazao e duplicata da pmon_alvo
    % Pega os nomes das tags da struct
    tagNames = fieldnames(tags);

    % Abre o arquivo para escrita
    fileID = fopen(nome_arquivo, 'w');

    % Escreve a primeira linha (valores das tags)
    for i = 1:length(tagNames)
        fprintf(fileID, '%s', tags.(tagNames{i}));
        if i < length(tagNames)
            fprintf(fileID, ';');
        end
    end
    fprintf(fileID, '\n');

    % Escreve a segunda linha (valores numéricos)
    for i = 1:length(dados)
        fprintf(fileID, '%.6f', dados(i));  % usando 6 casas decimais, ajuste conforme necessário
        if i < length(dados)
            fprintf(fileID, ';');
        end
    end

    % Fecha o arquivo
    fclose(fileID);
end

function deletar_arquivo_saidas_ddmpc(nome_arquivo)
    if exist(nome_arquivo, 'file')
        try
            delete(nome_arquivo);
        catch ME
            disp(strcat('erro na leitura do arquivo de saidas', ME))
        end
    end
end


function [variaveis] = ler_saidas_ddmpc(nome_arquivo)
    variaveis = [];
    % Loop para esperar o arquivo existir
    if exist(nome_arquivo, 'file')
        try
            % Tenta ler o arquivo
            dados = readtable(nome_arquivo, 'Delimiter', ';');

            % Se conseguiu ler, tenta deletar
            %delete(nome_arquivo);

            % Se chegou aqui, conseguiu ler e deletar
        catch ME
            disp(strcat('erro na leitura do arquivo de saidas', ME))
        end
    else
        return
    end


    % Criar estrutura para armazenar as variáveis
    variaveis = struct();

    variaveis.timestamp = dados.timestamp;
    variaveis.data_hora = dados.data_hora;
    variaveis.feasible = dados.feasible;
    variaveis.iteracoes = dados.iteracoes;
    variaveis.tempo_solver = dados.tempo_solver;
    variaveis.u0_0 = dados.u0_0;
    variaveis.u0_1 = dados.u0_1;
    variaveis.du0_0 = dados.du0_0;
    variaveis.du0_1 = dados.du0_1;
    variaveis.soma_delta_freq = dados.soma_delta_freq;
    variaveis.jy = dados.jy;
    variaveis.ju = dados.ju;
    variaveis.jr = dados.jr;
    variaveis.jx = dados.jx;
    variaveis.ysp_0 = dados.ysp_0;
    variaveis.ysp_1 = dados.ysp_1;

    % Extrair variáveis erro_x
    colunas_erro_x = startsWith(dados.Properties.VariableNames, 'erro_x_');
    nomes_erro_x = dados.Properties.VariableNames(colunas_erro_x);
    erro_x = zeros(1, sum(colunas_erro_x));

    for i = 1:length(nomes_erro_x)
        erro_x(i) = dados.(nomes_erro_x{i})(1);
    end
    variaveis.erro_x = erro_x;

    % Extrair variáveis erro_y
    colunas_erro_y = startsWith(dados.Properties.VariableNames, 'erro_y_');
    nomes_erro_y = dados.Properties.VariableNames(colunas_erro_y);
    erro_y = zeros(1, sum(colunas_erro_y));

    for i = 1:length(nomes_erro_y)
        erro_y(i) = dados.(nomes_erro_y{i})(1);
    end
    variaveis.erro_y = erro_y;

    % Extrair variáveis xk
    colunas_xk = startsWith(dados.Properties.VariableNames, 'xk_');
    nomes_xk = dados.Properties.VariableNames(colunas_xk);
    xk = zeros(1, sum(colunas_xk));

    for i = 1:length(nomes_xk)
        xk(i) = dados.(nomes_xk{i})(1);
    end
    variaveis.xk = xk;

    % Extrair variáveis uk
    colunas_uk = startsWith(dados.Properties.VariableNames, 'uk_');
    nomes_uk = dados.Properties.VariableNames(colunas_uk);
    uk = zeros(1, sum(colunas_uk));

    for i = 1:length(nomes_uk)
        uk(i) = dados.(nomes_uk{i})(1);
    end
    variaveis.uk = uk;

    fprintf('Arquivo lido e removido com sucesso!\n');
end

function [variaveis] = ler_saidas_ddmpc_quando_modificado(nome_arquivo)
    persistent ultima_modificacao;
    variaveis = [];
    
    % Loop infinito até que o arquivo seja criado ou modificado
    while true
        % Se o arquivo não existe, continua esperando
        if ~exist(nome_arquivo, 'file')
            disp('aguardando arquivo de saida ddmpc')
            pause(0.5);  % Pausa para não sobrecarregar o CPU
            continue;
        end
        
        % Obtém informações do arquivo
        info_arquivo = dir(nome_arquivo);
        modificacao_atual = info_arquivo.datenum;
        
        % Se é primeira execução (arquivo acabou de ser criado) OU arquivo foi modificado
        if isempty(ultima_modificacao) || modificacao_atual > ultima_modificacao
            try
                % Tenta ler o arquivo
                dados = readtable(nome_arquivo, 'Delimiter', ';');
                % Atualiza o tempo da última modificação
                ultima_modificacao = modificacao_atual;
                break;  % Sai do loop após leitura bem-sucedida
            catch ME
                disp(strcat('erro na leitura do arquivo de saidas', ME))
                pause(0.5);  % Pausa antes de tentar novamente
                continue;
            end
        end
        
        % Se arquivo existe mas não foi modificado, espera
        pause(0.1);
    end
    
    % Criar estrutura para armazenar as variáveis
    variaveis = struct();
    variaveis.timestamp = dados.timestamp;
    variaveis.data_hora = dados.data_hora;
    variaveis.feasible = dados.feasible;
    variaveis.iteracoes = dados.iteracoes;
    variaveis.tempo_solver = dados.tempo_solver;
    variaveis.u0_0 = dados.u0_0;
    variaveis.u0_1 = dados.u0_1;
    variaveis.du0_0 = dados.du0_0;
    variaveis.du0_1 = dados.du0_1;
    variaveis.soma_delta_freq = dados.soma_delta_freq;
    variaveis.jy = dados.jy;
    variaveis.ju = dados.ju;
    variaveis.jr = dados.jr;
    variaveis.jx = dados.jx;
    variaveis.ysp_0 = dados.ysp_0;
    variaveis.ysp_1 = dados.ysp_1;

    % Extrair variáveis erro_x
    colunas_erro_x = startsWith(dados.Properties.VariableNames, 'erro_x_');
    nomes_erro_x = dados.Properties.VariableNames(colunas_erro_x);
    erro_x = zeros(1, sum(colunas_erro_x));
    for i = 1:length(nomes_erro_x)
        erro_x(i) = dados.(nomes_erro_x{i})(1);
    end
    variaveis.erro_x = erro_x;

    % Extrair variáveis erro_y
    colunas_erro_y = startsWith(dados.Properties.VariableNames, 'erro_y_');
    nomes_erro_y = dados.Properties.VariableNames(colunas_erro_y);
    erro_y = zeros(1, sum(colunas_erro_y));
    for i = 1:length(nomes_erro_y)
        erro_y(i) = dados.(nomes_erro_y{i})(1);
    end
    variaveis.erro_y = erro_y;

    % Extrair variáveis xk
    colunas_xk = startsWith(dados.Properties.VariableNames, 'xk_');
    nomes_xk = dados.Properties.VariableNames(colunas_xk);
    xk = zeros(1, sum(colunas_xk));
    for i = 1:length(nomes_xk)
        xk(i) = dados.(nomes_xk{i})(1);
    end
    variaveis.xk = xk;

    % Extrair variáveis uk
    colunas_uk = startsWith(dados.Properties.VariableNames, 'uk_');
    nomes_uk = dados.Properties.VariableNames(colunas_uk);
    uk = zeros(1, sum(colunas_uk));
    for i = 1:length(nomes_uk)
        uk(i) = dados.(nomes_uk{i})(1);
    end
    variaveis.uk = uk;

    fprintf('Arquivo lido com sucesso!\n');
end