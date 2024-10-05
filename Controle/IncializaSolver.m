function SolucaoOtimizador = IncializaSolver(EstruturaSolver,Hp,Hc,Qy,Qu,R,ny,nu,nx,ModeloPreditor,MatrizSimuladorVazao,MatrizLimitesDinamicos)
% Parametros desta função:
% EstruturaSolver =1/2 indica o tipo de modelo usado para o preditor do MPC: ESN/LSTM
% Hp = Horizonte de predição
% Hc= Horizonte de controle
% Qy = Matriz diagonal para ponderação das variáveis controladas
% Qu = Matriz diagonal para ponderação dos alvos desejados
% R =  Matriz diagnal para ponderação das variáveis manipuladas ao longo de todo o horizonte de controle
%  ny= Número de variáveis controladas (no caso, 2: PSuc e PChegada)
%  nu=Número de variáveis manipuladas (no caso, 2: Freq e PMonAlvo)
%  nx = Número de variáveis coletadas do processo (no caso, 11=10+Vazão)
% ModeloPreditor = rede utilizada como preditor interno ao controlador
% MatrizSimuladorVazao = tabela do Simulador Petrobras (só as colunas de Freq,PChegada e Vazão), necessárias para a estimação da vazão
% MatrizLimitesDinamicos = tabela com os limites Max/Min de todas as variáveis em função da frequência (com resolução de 0.1Hz) 

    import casadi.*                              % Importa a biblioteca para definição de expressões matemáticas simbólicas no Casadi

    %% ========================Define parâmetros simbolicos (Casadi) gerais para o problema de otimização=====================================
    X =     MX.sym('X',nx,Hp+1);                           % Predição dos estados sobre o horizonte Hp  
    du =    MX.sym('du',Hc*nu,1);                          % Incrementos do controle sobre o horizonte Hc (Variável de decisão)
    Du =    [du;zeros(nu*(Hp-Hc),1)];                    % Sequencia de ação de controle
    ysp =   MX.sym('ysp',ny,1);                              % Set-point otimo calculado pelo otimizador (Variável de decisão)
    VarControladas = MX.sym('X_MPC',nx);       % Cria uma função de saída (h) em função dos estados X    
    h=Function('h',{VarControladas},{[VarControladas(1);VarControladas(2)]}); % Define os dois primeiros estados como controladas (Psuc e Pcheg) para o solver comparar com setpoint (ysp)

    %% ========================Escolha o modeloPreditor que será usando no MPC (1=ESN, 2=LSTM)  =====================================
    switch EstruturaSolver
        case 1
            disp('Usando uma estrutura ESN como preditor para o MPC');
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

            Freq_sym = MX.sym('Freq_sym',1);
            Press_sym = MX.sym('Press_sym',1);
            Interpola_casadi_vazao_sym = Interpola_casadi_vazao(Freq_sym, Press_sym,MatrizSimuladorVazao);
            f = Function('f', {Freq_sym, Press_sym}, {Interpola_casadi_vazao_sym});

    %         Limites_sym=MatrizLimitesDinamicos(MatrizLimitesDinamicos(:,1)==Freq_sym,2:end);   % Extrai restrições das variáveis do processo, extraindo a 1a coluna = Freq
    %        f1=Function('f', {Freq_sym}, {Limites_sym});
%             f1=Function('f', {Freq_sym}, {MatrizLimitesDinamicos(MatrizLimitesDinamicos(:,1)==Freq_sym,2:end)});

                    %Define a função objetivo (fob) de forma recursiva ao longo de Hp passos, utilizando o modelo preditor para otimizar as variáveis de controle, considerando as restrições do processo.
            for k=1:Hp
                uk_1 = uk_1 + Du((k-1)*nu+1:k*nu);           % define variável simbólica para soma dos incrementos de controle
                ym = h(X(:,k+1));                                           % define variável simbólica que será controlada utilizando a função de saída (h) definida anteriomente 
                fob=(ym-ysp+erro)'*Qy*(ym-ysp+erro)+du'*R*du+(uk_1-uRTO)'*Qu*(uk_1-uRTO);            % define a função objetivo proposta
                u = [uk_1;P(1:nx)];                                         % Define uma matriz para armazenar as variáveis de entrada no modeloPreditor
                ukk = normaliza_entradas(u);                       % Normaliza as variáveis para entrada no modeloPreditor
                x_ESN = ModeloPreditor.data.Wrr*ESNdataa0 + ModeloPreditor.data.Wir*ukk + ModeloPreditor.data.Wbr;  %usar o modeloPreditor(ESN) para fazer a predição
                next_state = (1-ModeloPreditor.data.gama)*ESNdataa0 + ModeloPreditor.data.gama*tanh(x_ESN);         % Atualisa estado da ESN
                a_wbias = [1.0;next_state];                                                                         % 
                yn = ModeloPreditor.data.Wro*a_wbias;                                                   % Variáveis preditas pela rede atualizada
                y_esn_pred = desnormaliza_predicoes(yn);                                              % desnormaliza as variáveis de saída no modeloPreditor
                VazaoEstimada=f(uk_1(1), y_esn_pred(2)*1.019716);   % Com base nas entradas (Freq e PChegada em Kgf/cm2), estima vazão
                y_esn_pred=[y_esn_pred; VazaoEstimada];                         % Insere vazão como mais uma variável predita

    %             Restricoes=MatrizLimitesDinamicos(MatrizLimitesDinamicos(:,1)==uk_1(1),:);    % Extrai restricões pela tabela já calculada

                g=[g;X(:,k+1)-y_esn_pred];   % Define variável simbólica para compor as restrições nos LimitesInferior e LimiteSuperior(lbg<g(x)<ubg)                                                                     
            end
        otherwise
            error('EstruturaSolver inválida. Selecione um valor válido.');
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
    SolucaoOtimizador = nlpsol('SolucaoOtimizador','ipopt', nlp,options); % Define o Interior Point OPTimizer (ipopt) para resolver o problema de otimização não linear (nlp)
end
%% =======================   FIM DA FUNÇÃO PRINCIPAL  ===========================================
