%%  IMPLEMENTAÇÃO MPC USANDO A BIBOIOTECA CASADI
classdef casadi_block_Control < matlab.System & matlab.system.mixin.Propagates
    properties (DiscreteState)
    end
    properties (Access = private)                       % Criação das variáveis que vão compor o OBJETO
        casadi_solver                                            % Criação do solver Casadi
        x0                                                                % Criação da variável para guardar os estados para operação do MPC
        contador                                                     % Criação da variável para guardar o contador de loops - define momentro de atuação do MPC
        ModeloPreditor                                           % Criação da variável para guardar modelo de preditor do processo e que será utilizada pelo solver para a predição
        MatrizLimitesDinamicos                            % Tabela com resultados dos limites Max/Min de proteção dinâmica para todas as frequências
        MatrizSimuladorVazao                                         % Tabela do simulador Petrobras para Interpolação
        limMax_casadi
        limMin_casadi
        TabelaLimitesDinamicos
            
        %        BufferLSTM   %%%   ?????                  % Tentar fazer Buffer. Usa o mesmo nome para Buffer ESN(data.a0) e da LSTM
    end
%%======================================================
    methods (Access = protected)
        %% ============   Ajuste das entradas
        function num = getNumInputsImpl(~)        % Nùmero de pinos de entradas do bloco CASADI (ver no Simulink)
            num = 14;                                               % São 14 pinos
            %    13 = 10 variáveis do processo + VazãoEstimada + 2 manipuladas (leitura da Freq e da PMon aplicadas ao processo)
            %     1  = clock = tempo de simulação dado pelo Simulink
            %     2  = Alvos = Valores alvo para as manipuladas (Freq e PMon)
            %     2  = dumax = Delta U máximo que pode ser praticado em cada uma das manipuladas  
            %     2  = dumin = Delta U mínmo que pode ser praticado em cada uma das manipuladas  
            %     1  = Hp = Horizonte de predição do controlador MPC
            %     1  = Hc = Horizonte de controle MPC
            %     2  = Qy = Matriz de supressão das variáveis controladas
            %     2  = R = Matriz de supressão das variáveis manipuladas
            %     2 = Qu = Matriz de supressão dos alvos definidos pelo usuário
            %     1 = Amostra MPC = Taxa de amstragem proporcional do MPC em relação aos dados de amostragem do processo
            %     1 = Umax (Freq e PmonAlvo Máximos)
            %     1 = Umin (Freq e PmonAlvo Mínimos)
            %     22 restrições dinâmicas (max/min das 10 variáveis do processo + max/min da vazão)

        end
        %===============        
        function sz1 = getInputSizeImpl(~)            % Organização das entradas do bloco Casadi
            sz1=14; 
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
            % num = 6;     %Poderia ser NUM=6 e colocarmos a implementação na forma   [1;1;2;2;2;11]
        end
        %===============        
        function sz1 = getOutputSizeImpl(~)         % Organização da saida do bloco Casadi
           sz1 = 1+1+2+2+2+11;                            % Dimensão será de 19
           % sz1 = [1;1;2;2;2;11];                            % Dimensão total será de 19, se quiser colocar na forma de 6 saídas
           % 1 = Informação de realizável (feasible) vindo do otimizador. (1/0) Indica se deu tempo de achar solução ótima
           % 1 = Tempo gasto pelo otimizador 
           % 2 = Valores calculados para as manipuladas Freq. e PMonAlvo
           % 2 = Variações ótimas calculadas para as manipuladas (Delta U de Freq e PMonAlvo). Será útil para visualizar a velocidade das alterações
           % 2 = Álvos ótimos calculados para a PSuc e PChegada
           % 11 = Pedições do Processo (só para visualização), incluindo a vazão
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
 %% ================     Inicialização dos parâmetros - só passa aqui uma única vez (equivale a Flag=0 na SFunction)
        function setupImpl(obj,~)
            import casadi.*                                                     % Importação da biblioteca Casadi (tem de estar no path)
            obj.casadi_solver = [];                                          % Definilção do Solver Casadi no objeto
            obj.x0 =  evalin('base','InicializaMPC');               % Condição inicial para operação do MPC  
            obj.contador = -1;                                                 % Contador para indicar passos/momentos de atuação do controle MPC (-1 para contagem de tempo sincronizar com passos de 30)
            obj.ModeloPreditor = evalin('base', 'ModeloPreditor');   % Modelos do preditor que será usada pelo MPC 
            
            % Tabelas para cálculos das proteções dinâmicas
            TabelaLimitesDinamicos =evalin('base', 'TabelaLimitesDinamicos');       % Tabela com limites Max/Min de todas as variáveis em todas as frequências (com resolução de 0,1)
            obj.TabelaLimitesDinamicos =TabelaLimitesDinamicos;
            obj.MatrizLimitesDinamicos = table2array(TabelaLimitesDinamicos(:, [1,3:end])); %cortando a coluna LIMITES
            TabelaSimulador=evalin('base', 'TabSimulador'); 
            obj.MatrizSimuladorVazao = table2array(TabelaSimulador(:,1:3));
%             EstruturaSolver = obj.ModeloPreditor.data.tipo;       Se for  LSTM
%             if EstruturaSolver==2
%                 [~,obj.BufferLSTM] = CarregaModeloLSTM(obj.ModeloPreditor);
%             end

        end
%% ================  Contas de atualização -  Equivale a Flag=2 na SFunction)
        function Saida = stepImpl(obj,DadosProcesso,t,Alvos,dumax,dumin,Hp,Hc,Qy,R,Qu,PassoMPC,umax,umin,Restricoes)
            disp(strcat("Simulação MPC em ",num2str(t)," s      Passo MPC = ",num2str(PassoMPC)))
             EstruturaSolver = obj.ModeloPreditor.data.tipo;     % Extrai tipo do preditor (1=ESN; 2 = LSTM)
            %% ===================== Configuração inicial do MPC  ========================================
            [ ny, nu, nx] = DimensoesVariaveisProcesso(Qy,Qu,DadosProcesso);
            UProcesso=DadosProcesso(nx+1:nx+nu);  % Extrai ações (Frequencia e PMonAlvo) aplicadas ao processo
            DadosProcesso=DadosProcesso(1:nx);      % Extrai apenas as nx variáveis com dados do processo
  
            EntradasESN_Normalizadas = normaliza_entradas([UProcesso;DadosProcesso]);   % Normaliza entradas provenientes do processo (observar que a função nada faz com a vazão)
            if t==0        % Assegura inicialização do solver e esquenta a ESN, caso esta seja o tipo do preditor usado
%                 obj.casadi_solver = IncializaSolver(obj.ModeloPreditor.data.tipo,Hp,Hc,Qy,Qu,R,ny,nu,nx,obj.ModeloPreditor); % cria o solver (otimizador) uma vez

                sol_args = IncializaSolver(obj.ModeloPreditor.data.tipo,Hp,Hc,Qy,Qu,R,ny,nu,nx,obj.ModeloPreditor,obj.MatrizSimuladorVazao, obj.MatrizLimitesDinamicos, dumax); % cria o solver (otimizador) uma vez
                obj.casadi_solver = sol_args.solucionador;

                if EstruturaSolver==1      % Se estratura do solver for uma ESN, precisa equentar
                    obj.ModeloPreditor.data.a0 = esquenta_ESN(obj.ModeloPreditor.data,EntradasESN_Normalizadas,1000); % Atualiza várias vezes o estado do reservátório para esquentar ESN
                end
            end
            
            %% ===================== Dados do processo e da predição ========================================
             PredicaoHorizonteHp=obj.x0(nx*(Hp)+1:nx*(Hp)+nx);                     % Busca variáveis preditas no horizonte Hp (futuro mais distante possível) 
            % Lembrar que as "nu" variáveis controladas (PSuc e PChegada) são as 2 primeiras do vetor de variáveis do processo
            erro =    DadosProcesso(1:nu)-PredicaoHorizonteHp(1:nu);            % Diferença entre a medição do processo e a última predição das variáveis controladas (dimensão=nu)
            ysp =     obj.x0(nx*(Hp+1)+Hc*nu+1:end);                                           % Resgata solução ótima (set-points) do MPC antes dada pelo otimizador
                        
            %% ===================== Empilha restrições para as variáveis do processo  =============================
            % Colocando na mesma organização do bloco de cálculo das restrições que foram recebidas nas entradas
             % Calculando as restrições com base nos valores atuais  recebidos na entrada do bloco
        
             % Calculando as restrições com base na tabela já prédefinida
            Freq=round(UProcesso(1),1);   % Arredonda em uma casa decimal - isso é feito considerando delta Freq Minimo =0.1 (Poderiamos interpolar, mas não parece mercer)

            % Como está fora de ordem para o Casadi, devemos colocar em ordem manualmente !!!
            % RestricoesMax=[Psuc(1); Pcheg(1); Pdif(1); Pdesc(1); Tmotor(1); Ctorque(1); CTotal(1); Tsuc(1); Vib(1); Tche(1); Vazao(1)];
            % RestricoesMin=[Psuc(2); Pcheg(2); Pdif(2); Pdesc(2); Tmotor(2); Ctorque(2); CTotal(2); Tsuc(2); Vib(2); Tche(2); Vazao(2)];
            
            Restricoes = obj.MatrizLimitesDinamicos(obj.MatrizLimitesDinamicos==Freq, :);
            RestricoesMax = Restricoes(1, 2:end)';
            RestricoesMin = Restricoes(2, 2:end)';
            %  Concatena limites das variáveis de processo (10) + restrições de manipulação (DeltaU) e das controladas
            %  variações nas manipuladas (Freq e PMonAlvo)  e faixa das  variáveis controladas (PSuc e PChegada)
            
            % Usando a faixa da PChegada como sendo os limites do mapa
            LimitesMax= [repmat(RestricoesMax,(Hp+1),1);repmat(dumax,Hc,1); [RestricoesMax(1) ;RestricoesMax(2)]];
            LimitesMin = [repmat(RestricoesMin,(Hp+1),1);repmat(-dumax,Hc,1); [RestricoesMin(1) ;RestricoesMin(2)]];
%             LimitesMax= [repmat(RestricoesMax,(Hp+1),1);repmat(dumax,Hc,1); [Psuc(1) ;30]];
%             LimitesMin = [repmat(RestricoesMin,(Hp+1),1);repmat(-dumax,Hc,1); [Psuc(2) ;30]];
%             % Usando a faixa da PChegada como sendo os limites máximos definidos pelo usuário
%             LimitesMax= [repmat(RestricoesMax,(Hp+1),1);repmat(dumax,Hc,1); [Psuc(1) ;umax(2) ]];
%             LimitesMin = [repmat(RestricoesMin,(Hp+1),1);repmat(-dumax,Hc,1); [Psuc(2) ;umin(2) ]];
 
%% ================ Empilha restrições para as variáveis manipuladas ==================================
            ManipuladasLowLimit = [repmat(zeros,nx*(Hp+1),1);repmat(umin,Hc,1)];   %Atribui em obj. as restrições para solver (limites inferiores)
            ManipuladasHighLimit = [repmat(zeros,nx*(Hp+1),1);repmat(umax,Hc,1)]; %Atribui em obj. as restrições para solver (limites superiores)
            

            %% ===================== %Parâmetros e atuação do solver ========================================
            obj.contador = obj.contador+1;     % Contador ajudará a saber se é para o Solver do Otimizador/Controlador atuar
            Tsolver=0;                 % Inicializa contados para calcular o tempo gasto com o Solver, quando for o caso !!
            DeltaU=[0;0];             % Assume variação nula. Será alterada ao passao pelo solver
            Feasible=0.5;            % Assumir padrão para indicar que não passou pelo Solver

           if EstruturaSolver==1
                par_solver =    [DadosProcesso;UProcesso;erro;Alvos;obj.ModeloPreditor.data.a0];    %DadosProcesso; UProcesso = última ação de controle; Erro entre medição e predição; Alvos ENG; Estado do reservatório da ESN
            end
            if EstruturaSolver==2
                u = [UProcesso;DadosProcesso];     %os regressores do processo (entradas e estados)
                uk_aux = normaliza_entradas(u);
                obj.BufferLSTM.pressao_succao_BCSS =      [obj.BufferLSTM.pressao_succao_BCSS(:,2:end)     [uk_aux(1);uk_aux(2); uk_aux(3)]];
                obj.BufferLSTM.pressao_chegada =          [obj.BufferLSTM.pressao_chegada(:,2:end)         [uk_aux(1);uk_aux(2); uk_aux(4)]];
                obj.BufferLSTM.pressao_descarga_BCSS =    [obj.BufferLSTM.pressao_descarga_BCSS(:,2:end)   [uk_aux(1);uk_aux(2); uk_aux(6)]];
                obj.BufferLSTM.temperatura_motor_BCSS =   [obj.BufferLSTM.temperatura_motor_BCSS(:,2:end)  [uk_aux(1);uk_aux(2); uk_aux(7)]];
                obj.BufferLSTM.corrente_torque_BCSS =     [obj.BufferLSTM.corrente_torque_BCSS(:,2:end)    [uk_aux(1);uk_aux(2); uk_aux(8)]];
                obj.BufferLSTM.corrente_total_BCSS =      [obj.BufferLSTM.corrente_total_BCSS(:,2:end)     [uk_aux(1);uk_aux(2); uk_aux(9)]];
                obj.BufferLSTM.temperatura_succao_BCSS =  [obj.BufferLSTM.temperatura_succao_BCSS(:,2:end) [uk_aux(1);uk_aux(2); uk_aux(10)]];
                obj.BufferLSTM.vibracao_BCSS =            [obj.BufferLSTM.vibracao_BCSS(:,2:end)           [uk_aux(1);uk_aux(2); uk_aux(11)]];
                obj.BufferLSTM.temperatura_chegada =      [obj.BufferLSTM.temperatura_chegada(:,2:end)     [uk_aux(1);uk_aux(2); uk_aux(12)]];
                P_Buffer = [obj.BufferLSTM.pressao_succao_BCSS(:);obj.BufferLSTM.pressao_chegada(:);obj.BufferLSTM.pressao_descarga_BCSS(:);obj.BufferLSTM.temperatura_motor_BCSS(:);obj.BufferLSTM.corrente_torque_BCSS(:);obj.BufferLSTM.corrente_total_BCSS(:);obj.BufferLSTM.temperatura_succao_BCSS(:);obj.BufferLSTM.vibracao_BCSS(:);obj.BufferLSTM.temperatura_chegada(:)];
                par_solver =    [DadosProcesso;UProcesso;erro;Alvos;P_Buffer];    %DadosProcesso; UProcesso = última ação de controle; Regressores; Alvos = objetivo econômico; obj.ModeloPreditor.data.a0 = estados atuais do reservatório no modelo ESN do controlador
            end
            
            %% ========= Conta do Solver para Galdir
            if (obj.contador==PassoMPC)      % Solver só entra no passo definido pelos parâmetros de Passo do MPC
                tStart=tic;                                                                     % Dispara contagem para medir o tempo do solver
                %solver_MPC=obj.casadi_solver('x0',obj.x0,'lbx', LimitesMin,'ubx', LimitesMax,'lbg', ManipuladasLowLimit, 'ubg', ManipuladasHighLimit, 'p', par_solver);
                solver_MPC=obj.casadi_solver('x0',obj.x0,'lbx', obj.limMin_casadi,'ubx', obj.limMax_casadi,'lbg', ManipuladasLowLimit, 'ubg', ManipuladasHighLimit, 'p', par_solver);
                Feasible=obj.casadi_solver.stats.success;             % Atualizar status do Feasible
                % Usa resposta do solver para atualizar variáveis
                obj.x0 = full(solver_MPC.x);                                       % Atualiza objeto com solução ótima no instante k (PredicaoHorizonteHp; Deltau_k, Ysp)
                ResultadoMPC=obj.x0;                                              % Vetor com resultados proveniente da otimização feita pelo MPC
                DeltaU=ResultadoMPC(nx*(Hp+1)+1:nx*(Hp+1)+nu);  % Extrai a ação de controle ótima (apenas para o instante k+1)
                for i=1:nu                                                                    % Para cada uma das variáveis manipuladas
                    DeltaU(i)=AvaliaDeltaU(DeltaU(i),dumin(i));        % Avalia se pode dar o passo e observa limites minimos (o máximo não precisa testar aqui pois o limite da ação vai ser tratado)
                end
                % Atualiza ação de controle com DeltaU resultante
                UProcesso = UProcesso + DeltaU;          % Aplica na saida a solução ótima (Uk + Delta Uk calculado pelo solver)
                
                % Extrai os novos set-points ótimos calculados
                ysp=ResultadoMPC(end-ny+1:end);                         % Extrai novos set-point ótimos (sempre no final do vetor que montamos para o MPC)
                
                obj.contador =0;                                                          % Reinicia contador para o passo do MPC
                Tsolver = toc(tStart);                                                   % Calcula tempo de atuação do solver
            end
            % Passando ou não pelo Solver, atualiza a ação de saida do controlador
            UProcesso=min(UProcesso,umax);     % Restringe as manipuladas (Freq e PMonAlvo) nos limites máximos pré-definidos
            UProcesso=max(UProcesso,umin);     % Restringe as manipuladas (Freq e PMonAlvo) nos limites mínimos pré-definidos
            
            %% ============    Mantém ESN atualizada   =======================
            % Atualiza o estado do reservátório (1 passo) para proxima predição
            if EstruturaSolver==1
                QtePassosESN=1;       % Quantidade de passos para manter a rede esquentada. No caso, apenas 1 passo
                 obj.ModeloPreditor.data.a0 = esquenta_ESN(obj.ModeloPreditor.data,EntradasESN_Normalizadas,QtePassosESN);
            end
            
            %% ============    Saida com todas as variáveis calculadas pelo otimizador   =========
            Saida = [Feasible;Tsolver;UProcesso;DeltaU;ysp;PredicaoHorizonteHp];      
           % Feasible (Dim=1), informação de realizável (feasible) vindo do otimizador. (1/0) Indica se deu tempo de achar solução ótima
           % Tsolver (Dim=1), tempo gasto pelo otimizador 
           % UProcesso (Dim=2), valores calculados para as manipuladas Freq. e PMonAlvo
           % DeltaU (Dim=2), variações (Delta U) ótimas calculadas para as manipuladas (Delta U de Freq e PMonAlvo). Será útil para visualizar a velocidade das alterações
           % ysp (Dim=2), alvos ótimos calculados para a PSuc e PChegada
           % PredicaoHorizonteHp (Dim=10), são as Pedições do Processo feitas pelo MPC
           
        end
    end
end
%===============================================================================
% ==================  FIM DO PROGRAMA PRINCIPAL % ==============================
%===============================================================================

%% ==================        Outras funções adicionar   ====================================
%% Função para ter as dimensões das variáveis em função dos parâmetros indicados pelo usuário na inicialização
function [ny, nu, nx] = DimensoesVariaveisProcesso(Qy,Qu,DadosProcesso);
    ny=length(Qy);         % Associa número de variáveis controladas (no caso, 2: PSuc e PChegada)
    nu=length(Qu);         % Associa número de variáveis manipuladas (no caso, 2: Freq e PMonAlvo)
    T=length(DadosProcesso);    % Tamanho do vetor de dados do processo, mas precisa descontar as manipuladas que vieram junto
    nx = T-nu;                 % Associa número de variáveis coletadas do processo (no caso, 11 = 13 menos as 2 manipuladas)
end
%=================================================================
%% Função para ajustar o DeltaU mínimo - ao menos enquanto não for tratada na formulação do solver
function RetornoDeltaU=AvaliaDeltaU(DeltaU,dumin);
    RetornoDeltaU=DeltaU;                                     % Retorna valor valor padrão calculado, sem correção pelo mínimo
    %     RetornoDeltaU=max([DeltaU,dumin]);       % Retorna valor limitado pelo deltaU minimo. Mas se deixar só assim, a ação de controle vai oscilar
    if dumin>0                                                             % Se foi especificado um deltaU minimo > 0  (se for=0, passa qq valor)
       CasasDecimais=countDecimals(dumin);        % Quantidade de casas decimais definidas para o limite minimo
       RetornoDeltaU=round(DeltaU,CasasDecimais);   % Retorna o DeltaU arredondado pelo numero de casas decimais com a resolução definida pelo valor minimo
    end
end
%   ============================================
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
%    ============================================

