function SolucaoOtimizador = IncializaSolver(EstruturaSolver,Hp,Hc,Qy,Qu,R,ny,nu,nx,ModeloPreditor,MatrizSimuladorVazao,MatrizLimitesDinamicos)
% Parametros desta fun��o:
% EstruturaSolver =1/2 indica o tipo de modelo usado para o preditor do MPC: ESN/LSTM
% Hp = Horizonte de predi��o
% Hc= Horizonte de controle
% Qy = Matriz diagonal para pondera��o das vari�veis controladas
% Qu = Matriz diagonal para pondera��o dos alvos desejados
% R =  Matriz diagnal para pondera��o das vari�veis manipuladas ao longo de todo o horizonte de controle
%  ny= N�mero de vari�veis controladas (no caso, 2: PSuc e PChegada)
%  nu=N�mero de vari�veis manipuladas (no caso, 2: Freq e PMonAlvo)
%  nx = N�mero de vari�veis coletadas do processo (no caso, 11=10+Vaz�o)
% ModeloPreditor = rede utilizada como preditor interno ao controlador
% MatrizSimuladorVazao = tabela do Simulador Petrobras (s� as colunas de Freq,PChegada e Vaz�o), necess�rias para a estima��o da vaz�o
% MatrizLimitesDinamicos = tabela com os limites Max/Min de todas as vari�veis em fun��o da frequ�ncia (com resolu��o de 0.1Hz) 

    import casadi.*                              % Importa a biblioteca para defini��o de express�es matem�ticas simb�licas no Casadi

    %% ========================Define par�metros simbolicos (Casadi) gerais para o problema de otimiza��o=====================================
    X =     MX.sym('X',nx,Hp+1);                           % Predi��o dos estados sobre o horizonte Hp  
    du =    MX.sym('du',Hc*nu,1);                          % Incrementos do controle sobre o horizonte Hc (Vari�vel de decis�o)
    Du =    [du;zeros(nu*(Hp-Hc),1)];                    % Sequencia de a��o de controle
    ysp =   MX.sym('ysp',ny,1);                              % Set-point otimo calculado pelo otimizador (Vari�vel de decis�o)
    VarControladas = MX.sym('X_MPC',nx);       % Cria uma fun��o de sa�da (h) em fun��o dos estados X    
    h=Function('h',{VarControladas},{[VarControladas(1);VarControladas(2)]}); % Define os dois primeiros estados como controladas (Psuc e Pcheg) para o solver comparar com setpoint (ysp)

    %% ========================Escolha o modeloPreditor que ser� usando no MPC (1=ESN, 2=LSTM)  =====================================
    switch EstruturaSolver
        case 1
            disp('Usando uma estrutura ESN como preditor para o MPC');
            %Par�metros simb�licos espec�ficos da ESN
            nx_ESN =      length(ModeloPreditor.data.Wir);   % Resgata o tamanho do reservat�rio da ESN utilizada como modelo preditor

            % P = quantidade de par�metros para o Solver. Os P par�metros s�o:
            % - DadosProcesso (dimens�o=nx)
            % - uk(entradas) (dimens�o=nu)
            % - Erro, sendo a diferen�a entre a medi��o do processo e a �ltima predi��o das vari�veis controladas (dimens�o=ny)
            % - Alvo dado pelo RTO (dimens�o=nu)
            % - Dados do reservat�rio da ESN utilizada pelo controlador (dimens�o=nx_ESN)
            P =           MX.sym('P',nx+nu+ny+nu+nx_ESN);    % qtd de par�metros para o Solver
            uk_1 =        P(nx+1:nx+nu);                                    % define vari�vel simb�lica das entradas (Freq. PmonAlvo)
            erro =        P(nx+nu+1:nx+nu+ny);                         % define vari�vel simb�lica para erro (DadosProcesso-Predi��oMPC) ->(Psuc e Pcheg)
            uRTO =        P(nx+nu+ny+1:nx+nu+ny+nu);         % define vari�vel simb�lica para Alvo (Freq. e PmonAlvo)
            ESNdataa0 =   P(nx+nu+ny+nu+1:end);              % define vari�vel simb�lica do reservat�rio da ESN
            g=[X(:,1)-P(1:nx)];                                                  % define variavel que vai empilhar as restri��es durante o Hp

            Freq_sym = MX.sym('Freq_sym',1);
            Press_sym = MX.sym('Press_sym',1);
            Interpola_casadi_vazao_sym = Interpola_casadi_vazao(Freq_sym, Press_sym,MatrizSimuladorVazao);
            f = Function('f', {Freq_sym, Press_sym}, {Interpola_casadi_vazao_sym});

    %         Limites_sym=MatrizLimitesDinamicos(MatrizLimitesDinamicos(:,1)==Freq_sym,2:end);   % Extrai restri��es das vari�veis do processo, extraindo a 1a coluna = Freq
    %        f1=Function('f', {Freq_sym}, {Limites_sym});
%             f1=Function('f', {Freq_sym}, {MatrizLimitesDinamicos(MatrizLimitesDinamicos(:,1)==Freq_sym,2:end)});

                    %Define a fun��o objetivo (fob) de forma recursiva ao longo de Hp passos, utilizando o modelo preditor para otimizar as vari�veis de controle, considerando as restri��es do processo.
            for k=1:Hp
                uk_1 = uk_1 + Du((k-1)*nu+1:k*nu);           % define vari�vel simb�lica para soma dos incrementos de controle
                ym = h(X(:,k+1));                                           % define vari�vel simb�lica que ser� controlada utilizando a fun��o de sa�da (h) definida anteriomente 
                fob=(ym-ysp+erro)'*Qy*(ym-ysp+erro)+du'*R*du+(uk_1-uRTO)'*Qu*(uk_1-uRTO);            % define a fun��o objetivo proposta
                u = [uk_1;P(1:nx)];                                         % Define uma matriz para armazenar as vari�veis de entrada no modeloPreditor
                ukk = normaliza_entradas(u);                       % Normaliza as vari�veis para entrada no modeloPreditor
                x_ESN = ModeloPreditor.data.Wrr*ESNdataa0 + ModeloPreditor.data.Wir*ukk + ModeloPreditor.data.Wbr;  %usar o modeloPreditor(ESN) para fazer a predi��o
                next_state = (1-ModeloPreditor.data.gama)*ESNdataa0 + ModeloPreditor.data.gama*tanh(x_ESN);         % Atualisa estado da ESN
                a_wbias = [1.0;next_state];                                                                         % 
                yn = ModeloPreditor.data.Wro*a_wbias;                                                   % Vari�veis preditas pela rede atualizada
                y_esn_pred = desnormaliza_predicoes(yn);                                              % desnormaliza as vari�veis de sa�da no modeloPreditor
                VazaoEstimada=f(uk_1(1), y_esn_pred(2)*1.019716);   % Com base nas entradas (Freq e PChegada em Kgf/cm2), estima vaz�o
                y_esn_pred=[y_esn_pred; VazaoEstimada];                         % Insere vaz�o como mais uma vari�vel predita

    %             Restricoes=MatrizLimitesDinamicos(MatrizLimitesDinamicos(:,1)==uk_1(1),:);    % Extrai restric�es pela tabela j� calculada

                g=[g;X(:,k+1)-y_esn_pred];   % Define vari�vel simb�lica para compor as restri��es nos LimitesInferior e LimiteSuperior(lbg<g(x)<ubg)                                                                     
            end
        otherwise
            error('EstruturaSolver inv�lida. Selecione um valor v�lido.');
    end
    %% ========================Define as matrizes auxiliares (Mtil, Itil) para o incremento de controle (a��o de controle) ao longo de Hc passos, no conjunto de restri��es (g)====================================
    Mtil=[];                         
    Itil=[];
    auxM=zeros(nu,Hc*nu);
    for in=1:Hc
        auxM=[eye(nu) auxM(:,1:(Hc-1)*nu)];
        Mtil=[Mtil;auxM];
        Itil=[Itil;eye(nu)];
    end
    % Conclui a inclus�o das restri��es nos estados e nas entradas.
    g = [g;Mtil*du+Itil*P(nx+1:nx+nu)]; 

    %% ========================Configura��o do otimizador====================================
    opt_variable=[X(:);du;ysp];                   %vari�veis calculadas pelo Solver(predi��o;incrementos de controle;set-point*) 
    nlp = struct('f',fob,'x',opt_variable,'g', g, 'p', P); %define a estrutura para problema de otimiza��o n�o linear (NLP, Nonlinear Programming), sendo: 1-fob, definida acima; 2-opt_variable: vari�veis de decis�o; 3-g, as restri��es do processo e 4-P, par�metros de entrada para Solver   

    %Configura��o espec�fica do otimizador
    options=struct;
    options.print_time=0;                         % Habilita tempo total de execu��o do solver deve ser impresso ou n�o.
    options.ipopt.print_level=1;                  % N�vel de detalhamento das mensagens de sa�da do IPOPT. Valores mais baixos resultam em menos mensagens (0 significa sem mensagens).
    options.ipopt.max_iter=100;                   % Especifica o n�mero m�ximo de itera��es que o solver deve executar antes de parar.
    options.ipopt.acceptable_tol=1e-4;            % Define a toler�ncia de converg�ncia do solver. Um valor menor indica uma solu��o mais precisa.
    options.ipopt.acceptable_obj_change_tol=1e-4; % Define uma toler�ncia aceit�vel para uma solu��o "boa o suficiente", �til para problemas onde a solu��o perfeita pode ser muito dif�cil de alcan�ar.
    SolucaoOtimizador = nlpsol('SolucaoOtimizador','ipopt', nlp,options); % Define o Interior Point OPTimizer (ipopt) para resolver o problema de otimiza��o n�o linear (nlp)
end
%% =======================   FIM DA FUN��O PRINCIPAL  ===========================================
