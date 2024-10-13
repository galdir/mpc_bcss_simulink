function sol_args = IncializaSolver(EstruturaSolver,Hp,Hc,Qy,Qu,R,ny,nu,nx,ModeloPreditor,matrizVazao,matrizLimitesDinamicos, dumax)
% Parametros desta fun��o:
% EstruturaSolver =1/2 indica o tipo de modelo usado para o preditor do MPC: ESN/LSTM
% Hp = Horizonte de predi��o
% Hc= Horizonte de controle
% Qy = Matriz diagonal para pondera��o das vari�veis controladas
% Qu = Matriz diagonal para pondera��o dos alvos desejados
% R =  Matriz diagnal para ponera��o das vari�veis manipuladas ao longo de todo o horizonte de controle
%  ny= N�mero de vari�veis controladas (no caso, 2: PSuc e PChegada)
%  nu=N�mero de vari�veis manipuladas (no caso, 2: Freq e PMonAlvo)
%  nx = N�mero de vari�veis coletadas do processo (no caso, 10)
% ModeloPreditor = rede utilizada como preditor interno ao controlador

import casadi.*                              % Importa a biblioteca para defini��o de express�es matem�ticas simb�licas no Casadi

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
        sol_args=struct;     % Inicializa vari�vel que vai armazenar a estrutura de argumentos
        
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
        erro =        P(nx+nu+1:nx+nu+ny);                         % define vari�vel simb�lica para erro (DadosProcesso-Predi��oMPC) ->(Psuc. Pcheg)
        uRTO =        P(nx+nu+ny+1:nx+nu+ny+nu);         % define vari�vel simb�lica para Alvo (Freq. e PmonAlvo)
        ESNdataa0 =   P(nx+nu+ny+nu+1:end);              % define vari�vel simb�lica do reservat�rio da ESN
        n_ESNdataa0 =size(ESNdataa0, 1);
        u = [uk_1;P(1:nx)];
        g=[X(:,1)-P(1:nx)];                              % define variavel que vai empilhar as restri��es durante o Hp
        
        %montando funcoes com expressoes casadi para acelerar o loop de
        %horizonte de predicao
        
        Press_sym = MX.sym('Press_sym',1);
        
        %entradas_sym = MX.sym('entradas_sym', nu+nx);
        %ESNdataa0_sym = MX.sym('reservatorio_sym', nx_ESN);
        %executa_predicao_ESN_vazao_sym = executa_predicao_ESN_vazao(entradas_sym, ESNdataa0_sym, ModeloPreditor, matrizVazao);
        %f_executa_predicao_ESN_vazao_sym = Function('f_predicao',{entradas_sym, ESNdataa0_sym}, {executa_predicao_ESN_vazao_sym});

        Freq_sym = MX.sym('Freq_sym',1);
        Interpola_casadi_vazao_sym = Interpola_casadi_vazao(Freq_sym, Press_sym, matrizVazao);
        f_Interpola_casadi_vazao_sym = Function('f_vazao', {Freq_sym, Press_sym}, {Interpola_casadi_vazao_sym});
        
        buscaRestricoesLimites_casadi_sym = buscaLimitesMatriz_casadi(matrizLimitesDinamicos, Freq_sym);
        f_buscaRestricoesLimites_casadi_sym = Function('f_lim', {Freq_sym}, {buscaRestricoesLimites_casadi_sym});
        
        %buscando limites dinamicos
        uk_11 = uk_1(1); % frequencia atual para buscar limites
        uk_11=floor(uk_11 * 10)/10; %arredondamento para uma casa decimal (em casadi nao existe round)
        limites = f_buscaRestricoesLimites_casadi_sym(uk_11);
        limMax = limites(1,:)';
        limMin = limites(2,:)';
        limMax_controladas = [limMax(1); limMax(2)];
        limMin_controladas = [limMin(1); limMin(2)];

        entradas_predicao_sym = MX.sym('entradas_predicao', nu+nx);
        n_ESNdataa0_sym = MX.sym('n_ESNdataa0', n_ESNdataa0);
        [predicoes_sym, novo_a0_sym] = executa_predicao_ESN_vazao(entradas_predicao_sym, n_ESNdataa0_sym, ModeloPreditor, f_Interpola_casadi_vazao_sym);
        f_executa_predicao_ESN_vazao_sym = Function('f_predicao', {entradas_predicao_sym, n_ESNdataa0_sym}, {predicoes_sym, novo_a0_sym});


        %Define a fun��o objetivo (fob) de forma recursiva ao longo de Hp passos, utilizando o modelo preditor para otimizar as vari�veis de controle, considerando as restri��es do processo.
        for k=1:Hp
            uk_1 = uk_1 + Du((k-1)*nu+1:k*nu);           % define vari�vel simb�lica para soma dos incrementos de controle
            ym = h(X(:,k+1));                                           % define vari�vel simb�lica que ser� controlada utilizando a fun��o de sa�da (h) definida anteriomente 
            fob=(ym-ysp+erro)'*Qy*(ym-ysp+erro)+du'*R*du+(uk_1-uRTO)'*Qu*(uk_1-uRTO);            % define a fun��o objetivo proposta
            %u = [uk_1;P(1:nx)];                                        % define uma matriz para armazenar as vari�veis de entrada no modeloPreditor
            u = [uk_1;X(:,k)];                                        % define uma matriz para armazenar as vari�veis de entrada no modeloPreditor
            

            %[y_esn_pred, ESNdataa0] = executa_predicao_ESN_vazao(u, ESNdataa0, ModeloPreditor, f_Interpola_casadi_vazao_sym);
            [y_esn_pred, ESNdataa0] = f_executa_predicao_ESN_vazao_sym(u, ESNdataa0);

                       
            g=[g;X(:,k+1)-y_esn_pred];   % Define vari�vel simb�lica para atender as restri��es nos LimitesInferior e LimiteSuperior(lbg<g(x)<ubg)                                                                     
            
            %buscando limites dinamicos
            uk_11 = uk_1(1); % frequencia atual para buscar limites
            uk_11=floor(uk_11 * 10)/10; %arredondamento para uma casa decimal (em casadi nao existe round)
            
            limites = f_buscaRestricoesLimites_casadi_sym(uk_11);

            limMax_atual = limites(1,:)';
            limMin_atual = limites(2,:)';
            limMin = [limMin; limMin_atual];
            limMax = [limMax; limMax_atual];

        
        end
        limMax = [limMax; repmat(dumax,Hc,1); limMax_controladas];
        limMin = [limMin; repmat(-dumax,Hc,1); limMin_controladas];
    case 2
        disp('Usando uma estrutura LSTM como preditor para o MPC');
        [ModeloLSTM,Buffer] = CarregaModeloLSTM(ModeloPreditor);
        nx_LSTM = size([Buffer.pressao_succao_BCSS(:);Buffer.pressao_chegada(:);Buffer.pressao_descarga_BCSS(:);Buffer.temperatura_motor_BCSS(:);Buffer.corrente_torque_BCSS(:);Buffer.corrente_total_BCSS(:);Buffer.temperatura_succao_BCSS(:);Buffer.vibracao_BCSS(:);Buffer.temperatura_chegada(:)],1); % define o tamanho do Buffer da LSTM
        P =      MX.sym('P',nx+nu+ny+nu+nx_LSTM);   % qtd de par�metros para o Solver(DadosProcesso,uk(entradas),erro(mismach),Alvo,Dados do Buffer LSTM)
        uk_1 =   P(nx+1:nx+ny);                                      % define vari�vel simb�lica das entradas (Freq. PmonAlvo)
        erro =   P(nx+ny+1:nx+ny+nu);                           % define vari�vel simb�lica para erro (DadosProcesso-Predi��oMPC) ->(Psuc. Pcheg)
        uRTO =   P(nx+ny+nu+1:nx+ny+nu+nu);            % define vari�vel simb�lica para Alvo (Freq. PmonAlvo)
        AtualizaBuffer = P(nx+ny+nu+nu+1:end);          % define vari�vel simb�lica do Buffer da LSTM
        
        %Converte o vetor "AtualizaBuffer(nx_LSTM,1) em matrizes para as seguintes vari�veis (Varnames)
         sizes = [3, 6; 3, 6; 3, 6;...
                 3, 8; 3, 6; 3, 6;...
                 3, 8; 3, 6; 3, 8];
        Varnames = {'pressao_succao_BCSS','pressao_chegada','pressao_descarga_BCSS',...
                    'temperatura_motor_BCSS','corrente_torque_BCSS','corrente_total_BCSS',...
                    'temperatura_succao_BCSS','vibracao_BCSS','temperatura_chegada'};
        start_idx = 1;                                          % Inicializando o �ndice para percorrer em AtualizaBuffer
        for posicao = 1:length(Varnames)                     % Loop para criar as 9 matrizes referente aos estados do processo
            rows = sizes(posicao, 1);
            cols = sizes(posicao, 2);
            num_elements = rows * cols;
            sub_matrix = reshape(AtualizaBuffer(start_idx:start_idx + num_elements - 1), [rows, cols]); % Extrair a parte correspondente ao vetor AtualizaBuffer
            eval([Varnames{posicao}, ' = sub_matrix;']);     % Atribuir a sub-matriz a uma vari�vel com o nome correspondente
            start_idx = start_idx + num_elements;   % Atualizar o �ndice inicial para o pr�ximo bloco
        end
            g=[X(:,1)-P(1:nx)];                                   % define variavel que vai empilha as restri��es durante o Hp
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
options.ipopt.acceptable_tol=1e-4;            % Define a toler�ncia de converg�ncia do solver. Um valor menor indica uma solu��o mais precisa.
options.ipopt.acceptable_obj_change_tol=1e-4; % Define uma toler�ncia aceit�vel para uma solu��o "boa o suficiente", �til para problemas onde a solu��o perfeita pode ser muito dif�cil de alcan�ar.
SolucaoOtimizador = nlpsol('SolucaoOtimizador','ipopt', nlp,options); % Define o Interior Point OPTimizer (ipopt) para resolver o problema de otimiza��o n�o linear (nlp)
sol_args.solucionador = SolucaoOtimizador;
end



function [predicoes, novo_a0] = executa_predicao_ESN_vazao(entradas, ESNdataa0, modelo_ESN, f_matrizVazao_sym)
%%
% entradas � uma vetor coluna: frequencia_BCSS, pressao_montante_alvo, ...
%           pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
%           temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, ...
%           temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
%   
% modelo_ESN precisar ter: .data.Wrr, .data.Wir, .data.Wbr
%
% ESNdataa0 � o estado do reservatorio da esn ap�s a ultima predicao
%
% matrizVazao � a matriz com vazoes estimadas por frequencia e pressao de
% chegada
%
% saidas � um vetor coluna com o instante seguinte para frequencia_BCSS, pressao_montante_alvo:
%           pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
%           temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, ...
%           temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada,
%           vazaoOleo

[predicoes, novo_a0] = executa_predicao_ESN(entradas, ESNdataa0, modelo_ESN);
 
%vazaoOleo_estimada = Interpola_casadi_vazao(entradas(1), predicoes(2)*1.019716, matrizVazao);
vazaoOleo_estimada = f_matrizVazao_sym(entradas(1), predicoes(2)*1.019716);

predicoes = [predicoes; vazaoOleo_estimada];

end


function [predicoes, novo_a0] = executa_predicao_ESN(entradas, ESNdataa0, modelo_ESN)
%%
% entradas � uma vetor coluna: frequencia_BCSS, pressao_montante_alvo, ...
%           pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
%           temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, ...
%           temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
%   
% modelo_ESN precisar ter: .data.Wrr, .data.Wir, .data.Wbr
%
% ESNdataa0 � o estado do reservatorio da esn ap�s a ultima predicao
%
% saidas � um vetor coluna com o instante seguinte para frequencia_BCSS, pressao_montante_alvo:
%           pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
%           temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, ...
%           temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
 
entradas_normalizadas = normaliza_entradas(entradas); 
x_ESN = modelo_ESN.data.Wrr*ESNdataa0 + modelo_ESN.data.Wir*entradas_normalizadas + modelo_ESN.data.Wbr;  %usar o modeloPreditor(ESN) para fazer a predi��o
novo_a0 = (1-modelo_ESN.data.gama)*ESNdataa0 + modelo_ESN.data.gama*tanh(x_ESN);         % Atualisa estado da ESN
a_wbias = [1.0; novo_a0];                                                                         % 
predicoes_normalizadas = modelo_ESN.data.Wro*a_wbias;  

predicoes = desnormaliza_predicoes(predicoes_normalizadas);  

end

function controladas = busca_controladas(estados, matriz_h)
n_estados = size(estados, 1);
n_matriz_h_2 = size(matriz_h, 1);
if n_estados == n_matriz_h_2
    controladas = matriz_h * estados;
else
    error('matriz_h incompativel');
end

end