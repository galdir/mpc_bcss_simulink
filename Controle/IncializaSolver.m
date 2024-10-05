function SolucaoOtimizador = IncializaSolver(EstruturaSolver,Hp,Hc,Qy,Qu,R,ny,nu,nx,ModeloPreditor,MatrizSimuladorVazao,MatrizLimitesDinamicos,Matriz_h)
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
    
    EstadosMedidos=MX.sym('HdeX',nx,1); 
    h=Function('h',{EstadosMedidos},{Matriz_h*EstadosMedidos}); 
    
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

            Press_sym = MX.sym('Press_sym',1);
            Freq_sym = MX.sym('Freq_sym',1);

            Interpola_casadi_vazao_sym = Interpola_casadi_vazao(Freq_sym, Press_sym, MatrizSimuladorVazao);
            f_Interpola_casadi_vazao_sym = Function('f_vazao', {Freq_sym, Press_sym}, {Interpola_casadi_vazao_sym});

    %         Limites_sym=MatrizLimitesDinamicos(MatrizLimitesDinamicos(:,1)==Freq_sym,2:end);   % Extrai restrições das variáveis do processo, extraindo a 1a coluna = Freq
    %        f1=Function('f', {Freq_sym}, {Limites_sym});
%             f1=Function('f', {Freq_sym}, {MatrizLimitesDinamicos(MatrizLimitesDinamicos(:,1)==Freq_sym,2:end)});

            % Define a função objetivo (fob) de forma recursiva ao longo de Hp passos, utilizando o modelo preditor para otimizar as variáveis de controle, considerando as restrições do processo.
            for k=1:Hp
                uk_1 = uk_1 + Du((k-1)*nu+1:k*nu);           % define variável simbólica para soma dos incrementos de controle
                ym = h(X(:,k+1));                                           % define variável simbólica que será controlada utilizando a função de saída (h) definida anteriomente 
                fob=(ym-ysp+erro)'*Qy*(ym-ysp+erro)+du'*R*du+(uk_1-uRTO)'*Qu*(uk_1-uRTO);            % define a função objetivo proposta
                
                EstadosMedidos=P(1:nx);
                EntradaModeloPreditor = [uk_1;EstadosMedidos];                                         % Define uma matriz para armazenar as variáveis de entrada no modeloPreditor
 
                [y_esn_pred, ESNdataa0] = executa_predicao(EntradaModeloPreditor  , ESNdataa0, ModeloPreditor, f_Interpola_casadi_vazao_sym);
               

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
%%========================
function [predicoes, novo_a0] = executa_predicao(entradas, ESNdataa0, modelo_ESN, f_matrizVazao_sym)
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