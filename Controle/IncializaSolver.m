function sol_args = IncializaSolver(EstruturaSolver,Hp,Hc,Qy,Qu,R,ny,nu,nx,ModeloPreditor,matrizVazao,matrizLimitesDinamicos, dumax)
% Parametros desta função:
% EstruturaSolver =1/2 indica o tipo de modelo usado para o preditor do MPC: ESN/LSTM
% Hp = Horizonte de predição
% Hc= Horizonte de controle
% Qy = Matriz diagonal para ponderação das variáveis controladas
% Qu = Matriz diagonal para ponderação dos alvos desejados
% R =  Matriz diagnal para poneração das variáveis manipuladas ao longo de todo o horizonte de controle
%  ny= Número de variáveis controladas (no caso, 2: PSuc e PChegada)
%  nu=Número de variáveis manipuladas (no caso, 2: Freq e PMonAlvo)
%  nx = Número de variáveis coletadas do processo (no caso, 10)
% ModeloPreditor = rede utilizada como preditor interno ao controlador

import casadi.*                              % Importa a biblioteca para definição de expressões matemáticas simbólicas no Casadi

%% ========================Define parâmetros simbolicos (Casadi) gerais para o problema de otimização=====================================
X =     MX.sym('X',nx,Hp+1);                           % Predição dos estados sobre o horizonte Hp  
du =    MX.sym('du',Hc*nu,1);                          % Incrementos do controle sobre o horizonte Hc (Variável de decisão)
Du =    [du;zeros(nu*(Hp-Hc),1)];                    % Sequencia de ação de controle
ysp =   MX.sym('ysp',ny,1);                              % Set-point otimo calculado pelo otimizador (Variável de decisão)
VarControladas = MX.sym('X_MPC',nx);       % Cria uma função de saída (h) em função dos estados (Psuc e Pcheg) para controldas    
h=Function('h',{VarControladas},{[VarControladas(1);VarControladas(2)]}); % Define os dois primeiros estados como controladas (Psuc e Pcheg) para o solver comparar com setpoint (ysp)

%% ========================Escolha o modeloPreditor que será usando no MPC (1=ESN, 2=LSTM)  =====================================
switch EstruturaSolver
    case 1
        disp('Usando uma estrutura ESN como preditor para o MPC');
        sol_args=struct;     % Inicializa variável que vai armazenar a estrutura de argumentos
        
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
        erro =        P(nx+nu+1:nx+nu+ny);                         % define variável simbólica para erro (DadosProcesso-PrediçãoMPC) ->(Psuc. Pcheg)
        uRTO =        P(nx+nu+ny+1:nx+nu+ny+nu);         % define variável simbólica para Alvo (Freq. e PmonAlvo)
        ESNdataa0 =   P(nx+nu+ny+nu+1:end);              % define variável simbólica do reservatório da ESN
        u = [uk_1;P(1:nx)];
        g=[X(:,1)-P(1:nx)];                              % define variavel que vai empilhar as restrições durante o Hp
        
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
        
        buscaRestricoesLimites_casadi_sym = buscaLimitesTabela_casadi(matrizLimitesDinamicos, Freq_sym);
        f_buscaRestricoesLimites_casadi_sym = Function('f_lim', {Freq_sym}, {buscaRestricoesLimites_casadi_sym});
        
        %buscando limites dinamicos
        uk_11 = uk_1(1); % frequencia atual para buscar limites
        uk_11=floor(uk_11 * 10)/10; %arredondamento para uma casa decimal (em casadi nao existe round)
        limites = f_buscaRestricoesLimites_casadi_sym(uk_11);
        limMax = limites(1,:)';
        limMin = limites(2,:)';
        limMax_controladas = [limMax(1); limMax(2)];
        limMin_controladas = [limMin(1); limMin(2)];


        %Define a função objetivo (fob) de forma recursiva ao longo de Hp passos, utilizando o modelo preditor para otimizar as variáveis de controle, considerando as restrições do processo.
        for k=1:Hp
            uk_1 = uk_1 + Du((k-1)*nu+1:k*nu);           % define variável simbólica para soma dos incrementos de controle
            ym = h(X(:,k+1));                                           % define variável simbólica que será controlada utilizando a função de saída (h) definida anteriomente 
            fob=(ym-ysp+erro)'*Qy*(ym-ysp+erro)+du'*R*du+(uk_1-uRTO)'*Qu*(uk_1-uRTO);            % define a função objetivo proposta
            u = [uk_1;P(1:nx)];                                        % define uma matriz para armazenar as variáveis de entrada no modeloPreditor
            
            %resultado = full(f_executa_predicao_ESN_vazao_sym(u, ESNdataa0));
            %y_esn_pred = resultado(1);
            %ESNdataa0 = resultado(2);
            %[y_esn_pred, ESNdataa0] = executa_predicao_ESN(u, ESNdataa0, ModeloPreditor);
            [y_esn_pred, ESNdataa0] = executa_predicao_ESN_vazao(u, ESNdataa0, ModeloPreditor, f_Interpola_casadi_vazao_sym);
%             ukk = normaliza_entradas(u);                       % normaliza as variáveis para entrada no modeloPreditor            
%             x_ESN = ModeloPreditor.data.Wrr*ESNdataa0 + ModeloPreditor.data.Wir*ukk + ModeloPreditor.data.Wbr;  %usar o modeloPreditor(ESN) para fazer a predição
%             
%             ESNdataa0 = (1-ModeloPreditor.data.gama)*ESNdataa0 + ModeloPreditor.data.gama*tanh(x_ESN);         % Atualisa estado da ESN
%             a_wbias = [1.0;ESNdataa0];                                                                         % 
%             yn = ModeloPreditor.data.Wro*a_wbias;                                                   % Variáveis preditas pela rede atualizada
%             y_esn_pred = desnormaliza_predicoes(yn);                                              % desnormaliza as variáveis de saída no modeloPreditor
%             
            %VazaoEstimada=f_Interpola_casadi_vazao_sym(uk_1(1), y_esn_pred(2)*1.019716);   % Com base nas entradas (Freq e PChegada em Kgf/cm2), estima vazão
            %y_esn_pred=[y_esn_pred; VazaoEstimada];
                       
            g=[g;X(:,k+1)-y_esn_pred];   % Define variável simbólica para atender as restrições nos LimitesInferior e LimiteSuperior(lbg<g(x)<ubg)                                                                     
            
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
        P =      MX.sym('P',nx+nu+ny+nu+nx_LSTM);   % qtd de parâmetros para o Solver(DadosProcesso,uk(entradas),erro(mismach),Alvo,Dados do Buffer LSTM)
        uk_1 =   P(nx+1:nx+ny);                                      % define variável simbólica das entradas (Freq. PmonAlvo)
        erro =   P(nx+ny+1:nx+ny+nu);                           % define variável simbólica para erro (DadosProcesso-PrediçãoMPC) ->(Psuc. Pcheg)
        uRTO =   P(nx+ny+nu+1:nx+ny+nu+nu);            % define variável simbólica para Alvo (Freq. PmonAlvo)
        AtualizaBuffer = P(nx+ny+nu+nu+1:end);          % define variável simbólica do Buffer da LSTM
        
        %Converte o vetor "AtualizaBuffer(nx_LSTM,1) em matrizes para as seguintes variáveis (Varnames)
         sizes = [3, 6; 3, 6; 3, 6;...
                 3, 8; 3, 6; 3, 6;...
                 3, 8; 3, 6; 3, 8];
        Varnames = {'pressao_succao_BCSS','pressao_chegada','pressao_descarga_BCSS',...
                    'temperatura_motor_BCSS','corrente_torque_BCSS','corrente_total_BCSS',...
                    'temperatura_succao_BCSS','vibracao_BCSS','temperatura_chegada'};
        start_idx = 1;                                          % Inicializando o índice para percorrer em AtualizaBuffer
        for posicao = 1:length(Varnames)                     % Loop para criar as 9 matrizes referente aos estados do processo
            rows = sizes(posicao, 1);
            cols = sizes(posicao, 2);
            num_elements = rows * cols;
            sub_matrix = reshape(AtualizaBuffer(start_idx:start_idx + num_elements - 1), [rows, cols]); % Extrair a parte correspondente ao vetor AtualizaBuffer
            eval([Varnames{posicao}, ' = sub_matrix;']);     % Atribuir a sub-matriz a uma variável com o nome correspondente
            start_idx = start_idx + num_elements;   % Atualizar o índice inicial para o próximo bloco
        end
            g=[X(:,1)-P(1:nx)];                                   % define variavel que vai empilha as restrições durante o Hp
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
sol_args.solucionador = SolucaoOtimizador;
end



function [predicoes, novo_a0] = executa_predicao_ESN_vazao(entradas, ESNdataa0, modelo_ESN, f_matrizVazao_sym)
%%
% entradas é uma vetor coluna: frequencia_BCSS, pressao_montante_alvo, ...
%           pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
%           temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, ...
%           temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
%   
% modelo_ESN precisar ter: .data.Wrr, .data.Wir, .data.Wbr
%
% ESNdataa0 é o estado do reservatorio da esn após a ultima predicao
%
% matrizVazao é a matriz com vazoes estimadas por frequencia e pressao de
% chegada
%
% saidas é um vetor coluna com o instante seguinte para frequencia_BCSS, pressao_montante_alvo:
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
% entradas é uma vetor coluna: frequencia_BCSS, pressao_montante_alvo, ...
%           pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
%           temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, ...
%           temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
%   
% modelo_ESN precisar ter: .data.Wrr, .data.Wir, .data.Wbr
%
% ESNdataa0 é o estado do reservatorio da esn após a ultima predicao
%
% saidas é um vetor coluna com o instante seguinte para frequencia_BCSS, pressao_montante_alvo:
%           pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
%           temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, ...
%           temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
 
entradas_normalizadas = normaliza_entradas(entradas); 
x_ESN = modelo_ESN.data.Wrr*ESNdataa0 + modelo_ESN.data.Wir*entradas_normalizadas + modelo_ESN.data.Wbr;  %usar o modeloPreditor(ESN) para fazer a predição
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