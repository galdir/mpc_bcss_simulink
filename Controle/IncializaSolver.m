function SolucaoOtimizador = IncializaSolver(EstruturaSolver,Hp,Hc,q,r,qu,ny,nu,nx,ModeloPreditor)
import casadi.*                              % Importa a biblioteca para defini��o de express�es matem�ticas simb�licas no Casadi
%% ========================Define Matrizes=====================================
Qy = diag(q');                               %Matriz de pondera��o (minimizar o desvio) das vari�veis controladas (Psuc e Pcheg) em diagonal para FOB (conceito do MPC).
R =  diag(repmat(r',1,Hc));          %Matriz de supress�o  (ajustado para diminuir/aumentar o custo de controle) das vari�veis manipuladas (Freq. e PmonAlvo) em diagonal para FOB (conceito do MPC).
Qu = diag(qu');                              %Matriz de peso em diagonal dos Alvos (Freq. e PmonAlvo) para FOB.

%% ========================Define par�metros simbolicos (Casadi) gerais para o problema de otimiza��o=====================================
X =     MX.sym('X',nx,Hp+1);                           % Predi��o dos estados sobre o horizonte Hp  
du =    MX.sym('du',Hc*nu,1);                          % Incrementos do controle sobre o horizonte Hc (Vari�vel de decis�o)
Du =    [du;zeros(nu*(Hp-Hc),1)];                    % Sequencia de a��o de controle
ysp =   MX.sym('ysp',ny,1);                              % Set-point otimo calculado pelo otimizador (Vari�vel de decis�o)
VarControladas = MX.sym('X_MPC',nx);       % Cria uma fun��o de sa�da (h) em fun��o dos estados (Psuc e Pcheg) para controldas    
h=Function('h',{VarControladas},{[VarControladas(1);VarControladas(2)]}); % Define os dois primeiros estados como controladas (Psuc e Pcheg) para o solver comparar com setpoint (ysp)

%% ========================Escolha o modeloPreditor que ser� usando no MPC (1=ESN, 2=LSTM)  =====================================
switch EstruturaSolver
    case 1
        disp('Usando uma estrutura ESN como preditor para o MPC');
        %Par�metros simb�licos espec�ficos da ESN
        nx_ESN =      length(ModeloPreditor.data.Wir);   % define o tamanho do reservat�rio da ESN
        P =           MX.sym('P',nx+nu+ny+nu+nx_ESN);    % qtd de par�metros para o Solver(DadosProcesso,uk(entradas),erro(mismach),Alvo,Dados do reserv.(ESNdataa0))
        uk_1 =        P(nx+1:nx+ny);                                    % define vari�vel simb�lica das entradas (Freq. PmonAlvo)
        erro =        P(nx+ny+1:nx+ny+nu);                         % define vari�vel simb�lica para erro (DadosProcesso-Predi��oMPC) ->(Psuc. Pcheg)
        uRTO =        P(nx+ny+nu+1:nx+ny+nu+nu);         % define vari�vel simb�lica para Alvo (Freq. PmonAlvo)
        ESNdataa0 =   P(nx+ny+nu+nu+1:end);              % define vari�vel simb�lica do reservat�rio da ESN
        g=[X(:,1)-P(1:nx)];                                                  % define variavel que vai empilha as restri��es durante o Hp

        %Define a fun��o objetivo (fob) de forma recursiva ao longo de Hp passos, utilizando o modelo preditor para otimizar as vari�veis de controle, considerando as restri��es do processo.
        for k=1:Hp
            uk_1 = uk_1 + Du((k-1)*nu+1:k*nu);           % define vari�vel simb�lica para soma dos incrementos de controle
            ym = h(X(:,k+1));                            % define vari�vel simb�lica que ser� controlada utilizando a fun��o de sa�da (h) definida anteriomente 
            fob=(ym-ysp+erro)'*Qy*(ym-ysp+erro)+du'*R*du+(uk_1-uRTO)'*Qu*(uk_1-uRTO);            % define a fun��o objetivo proposta
            u = [uk_1(1);uk_1(2);P(1:nx)];                                                       % define uma matriz para armazenar as vari�veis de entrada no modeloPreditor
            ukk = normaliza_entradas(u);                                                         % normaliza as vari�veis para entrada no modeloPreditor
            x_ESN = ModeloPreditor.data.Wrr*ESNdataa0 + ModeloPreditor.data.Wir*ukk + ModeloPreditor.data.Wbr;  %usar o modeloPreditor(ESN) para fazer a predi��o
            next_state = (1-ModeloPreditor.data.gama)*ESNdataa0 + ModeloPreditor.data.gama*tanh(x_ESN);         % ||
            a_wbias = [1.0;next_state];                                                                         % ||
            yn = ModeloPreditor.data.Wro*a_wbias;                                                               % ||
            y_esn_pred = desnormaliza_predicoes(yn);                                                            % desnormaliza as vari�veis de sa�da no modeloPreditor
            g=[g;X(:,k+1)-y_esn_pred];                                                                          % define vari�vel simb�lica para compor as restri��es nos LimitesInferir e LimiteSuperior(lbg<g(x)<ubg)                                                                     
        end
    case 2
        disp('Usando uma estrutura LSTM como preditor para o MPC');
        [ModeloLSTM,Buffer] = CarregaModeloLSTM(ModeloPreditor);
        nx_LSTM = size([Buffer.pressao_succao_BCSS(:);Buffer.pressao_chegada(:);Buffer.pressao_descarga_BCSS(:);Buffer.temperatura_motor_BCSS(:);Buffer.corrente_torque_BCSS(:);Buffer.corrente_total_BCSS(:);Buffer.temperatura_succao_BCSS(:);Buffer.vibracao_BCSS(:);Buffer.temperatura_chegada(:)],1); % define o tamanho do Buffer da LSTM
        P =      MX.sym('P',nx+nu+ny+nu+nx_LSTM);   % qtd de par�metros para o Solver(DadosProcesso,uk(entradas),erro(mismach),Alvo,Dados do Buffer LSTM)
        uk_1 =   P(nx+1:nx+ny);                     % define vari�vel simb�lica das entradas (Freq. PmonAlvo)
        erro =   P(nx+ny+1:nx+ny+nu);               % define vari�vel simb�lica para erro (DadosProcesso-Predi��oMPC) ->(Psuc. Pcheg)
        uRTO =   P(nx+ny+nu+1:nx+ny+nu+nu);         % define vari�vel simb�lica para Alvo (Freq. PmonAlvo)
        AtualizaBuffer = P(nx+ny+nu+nu+1:end);      % define vari�vel simb�lica do Buffer da LSTM
        
        %Converte o vetor "AtualizaBuffer(nx_LSTM,1) em matrizes para as seguintes vari�veis (Varnames)
         sizes = [3, 6; 3, 6; 3, 6;...
                 3, 8; 3, 6; 3, 6;...
                 3, 8; 3, 6; 3, 8];
        Varnames = {'pressao_succao_BCSS','pressao_chegada','pressao_descarga_BCSS',...
                    'temperatura_motor_BCSS','corrente_torque_BCSS','corrente_total_BCSS',...
                    'temperatura_succao_BCSS','vibracao_BCSS','temperatura_chegada'};
        start_idx = 1;                                 % Inicializando o �ndice para percorrer em AtualizaBuffer
        for i = 1:length(Varnames)                     % Loop para criar as 9 matrizes referente aos estados do processo
            rows = sizes(i, 1);
            cols = sizes(i, 2);
            num_elements = rows * cols;
            sub_matrix = reshape(AtualizaBuffer(start_idx:start_idx + num_elements - 1), [rows, cols]); % Extrair a parte correspondente ao vetor AtualizaBuffer
            eval([Varnames{i}, ' = sub_matrix;']);      % Atribuir a sub-matriz a uma vari�vel com o nome correspondente
            start_idx = start_idx + num_elements;       % Atualizar o �ndice inicial para o pr�ximo bloco
        end
            g=[X(:,1)-P(1:nx)];                         % define variavel que vai empilha as restri��es durante o Hp
        for k=1:Hp
            uk_1 = uk_1 + Du((k-1)*nu+1:k*nu);
            ym = h(X(:,k+1));
            fob=(ym-ysp+erro)'*Qy*(ym-ysp+erro)+du'*R*du+(uk_1-uRTO)'*Qu*(uk_1-uRTO);
            Psuc =     ModeloLSTM.pressao_succao_BCSS.modelo_casadi.casadi_func        (pressao_succao_BCSS ,    ModeloLSTM.pressao_succao_BCSS.modelo_casadi.h0_input,          ModeloLSTM.pressao_succao_BCSS.modelo_casadi.c0_input);
            Pche =     ModeloLSTM.pressao_chegada.modelo_casadi.casadi_func            (pressao_chegada ,        ModeloLSTM.pressao_chegada.modelo_casadi.h0_input,              ModeloLSTM.pressao_chegada.modelo_casadi.c0_input);
            Pdesc =    ModeloLSTM.pressao_descarga_BCSS.modelo_casadi.casadi_func      (pressao_descarga_BCSS ,  ModeloLSTM.pressao_descarga_BCSS.modelo_casadi.h0_input,        ModeloLSTM.pressao_descarga_BCSS.modelo_casadi.c0_input);
            Tmor =     ModeloLSTM.temperatura_motor_BCSS.modelo_casadi.casadi_func     (temperatura_motor_BCSS , ModeloLSTM.temperatura_motor_BCSS.modelo_casadi.h0_input,       ModeloLSTM.temperatura_motor_BCSS.modelo_casadi.c0_input);
            Ctq =      ModeloLSTM.corrente_torque_BCSS.modelo_casadi.casadi_func       (corrente_torque_BCSS ,   ModeloLSTM.corrente_torque_BCSS.modelo_casadi.h0_input,         ModeloLSTM.corrente_torque_BCSS.modelo_casadi.c0_input);
            Ct =       ModeloLSTM.corrente_total_BCSS.modelo_casadi.casadi_func        (corrente_total_BCSS ,    ModeloLSTM.corrente_total_BCSS.modelo_casadi.h0_input,          ModeloLSTM.corrente_total_BCSS.modelo_casadi.c0_input);
            Ts =       ModeloLSTM.temperatura_succao_BCSS.modelo_casadi.casadi_func    (temperatura_succao_BCSS, ModeloLSTM.temperatura_succao_BCSS.modelo_casadi.h0_input,      ModeloLSTM.temperatura_succao_BCSS.modelo_casadi.c0_input);
            Vib =      ModeloLSTM.vibracao_BCSS.modelo_casadi.casadi_func              (vibracao_BCSS ,          ModeloLSTM.vibracao_BCSS.modelo_casadi.h0_input,                ModeloLSTM.vibracao_BCSS.modelo_casadi.c0_input);
            Tche =     ModeloLSTM.temperatura_chegada.modelo_casadi.casadi_func        (temperatura_chegada,     ModeloLSTM.temperatura_chegada.modelo_casadi.h0_input,          ModeloLSTM.temperatura_chegada.modelo_casadi.c0_input);
            Pdiff =    full(Pdesc(end))-full(Psuc(end));
            yn =       [full(Psuc(end));full(Pche(end));Pdiff;full(Pdesc(end));full(Tmor(end));full(Ctq(end));full(Ct(end));full(Ts(end));full(Vib(end));full(Tche(end))];
            y_lstm_pred = desnormaliza_predicoes(yn);
            g=[g;X(:,k+1)-y_lstm_pred];
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
options.ipopt.acceptable_tol=1e-8;            % Define a toler�ncia de converg�ncia do solver. Um valor menor indica uma solu��o mais precisa.
options.ipopt.acceptable_obj_change_tol=1e-8; % Define uma toler�ncia aceit�vel para uma solu��o "boa o suficiente", �til para problemas onde a solu��o perfeita pode ser muito dif�cil de alcan�ar.
SolucaoOtimizador = nlpsol('SolucaoOtimizador','ipopt', nlp,options); % Define o Interior Point OPTimizer (ipopt) para resolver o problema de otimiza��o n�o linear (nlp)



