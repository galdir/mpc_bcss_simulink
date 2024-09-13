function predicoes_tempo = calc_LSTMs(freq, abertura, ...
    p_chegada, p_succao, p_descarga, p_diferencial, ...
    temp_succao, temp_motor, c_torque, c_total, vibracao, p_mon_alvo, temperatura_chegada,  modelos, nome_conj_modelos)
    
    tStart = tic;  

% Função para cálculo das predições LSTM
    variaveisBCSS.frequencia_BCSS = freq;
    variaveisBCSS.choke_producao = abertura;
    variaveisBCSS.pressao_chegada = p_chegada;  
    variaveisBCSS.pressao_succao_BCSS = p_succao;  
    variaveisBCSS.pressao_descarga_BCSS = p_descarga;  
    variaveisBCSS.pressao_diferencial_BCSS = p_diferencial;  
    variaveisBCSS.temperatura_succao_BCSS = temp_succao;  
    variaveisBCSS.temperatura_motor_BCSS = temp_motor;  
    variaveisBCSS.corrente_torque_BCSS = c_torque;  
    variaveisBCSS.corrente_total_BCSS = c_total;  
    variaveisBCSS.vibracao_BCSS = vibracao;  
    variaveisBCSS.pressao_montante_alvo = p_mon_alvo; 
    variaveisBCSS.temperatura_chegada = temperatura_chegada; 
    
    predicoes_tempo = zeros(1, 11);

    variaveis_alvo = {'pressao_chegada', 'pressao_succao_BCSS', 'pressao_descarga_BCSS', 'pressao_diferencial_BCSS', ...
                            'temperatura_succao_BCSS', 'temperatura_motor_BCSS', 'corrente_torque_BCSS', 'corrente_total_BCSS', ...
                            'vibracao_BCSS', 'temperatura_chegada'};
    
    simTime = get_param(bdroot, 'SimulationTime');
    %disp(simTime);

    modelos_disponiveis = {};
    for i=1:length(modelos)
        modelos_disponiveis{i}= modelos{i}{1};
    end
    
    
    for i=1:length(variaveis_alvo)
        variavel = variaveis_alvo{i};
        [esta_presente, loc]  = ismember(variavel, modelos_disponiveis);
        if esta_presente
            %disp([variavel ' está presente no conjunto de modelos.']);
            modelo = modelos{loc}{2};

            %extrai configuracoes modelo lstm
            configuracoes_modelo_lstm = modelos{loc}{3};
            variaveisPreditoras = configuracoes_modelo_lstm.preditoras';
            variavelAlvo = configuracoes_modelo_lstm.variavel_predicao;
            num_amostrasPassadas = configuracoes_modelo_lstm.num_amostras_passadas;
            num_variaveis = size(variaveisPreditoras', 1);
            
            entrada_atual = modelos{loc}{4};           
            
            %caso primeira execucao, criando entradas iniciais
            if simTime==0
                condicoes_iniciais = zeros(1, num_variaveis);
                for ipred=1:length(variaveisPreditoras)
                    condicoes_iniciais(ipred) = variaveisBCSS.(variaveisPreditoras{ipred});
                end

                entrada_atual = repmat(condicoes_iniciais(:)', num_amostrasPassadas, 1)';
            else
                %pegando os valores atuais das preditoras e botando no fim
                %da entrada atual
                nova_entrada = zeros(num_variaveis, num_amostrasPassadas);
                nova_entrada(:, 1:end-1) = entrada_atual(:, 2:end);
                entrada_atual = nova_entrada;
                for ivar=1:length(variaveisPreditoras)
                    entrada_atual(ivar, end) = variaveisBCSS.(variaveisPreditoras{ivar});
                end
            end

            %armazenando entrada atual para proxima chamada desse modelo
            modelos{loc}{4} = entrada_atual; 
            assignin('base', nome_conj_modelos, modelos);

            %normalizando entradas
            entrada_normalizada = entrada_atual;
            for ivar = 1:length(variaveisPreditoras)
                entrada_normalizada(ivar, :) = normalizar_dado_BCS(entrada_normalizada(ivar, :), variaveisPreditoras{ivar});
            end

            if isfield(modelo,'modelo_casadi')
                predicao = full(predict_LSTM_Casadi(modelo.modelo_casadi, entrada_normalizada));
            else
                predicao = double(predict(modelo, entrada_normalizada));
            end
            
            predicao = desnormalizar_dado_BCS(predicao, variavelAlvo);                        

        else
            predicao = 0;
        end

        predicoes_tempo(i) = predicao;
    end

    if ~ismember('pressao_diferencial_BCSS', modelos_disponiveis)
        ipressao_descarga = find(strcmp('pressao_descarga_BCSS', variaveis_alvo));
        ipressao_succao = find(strcmp('pressao_succao_BCSS', variaveis_alvo));
        predicao = predicoes_tempo(ipressao_descarga) - predicoes_tempo(ipressao_succao);
        ipressao_diferencial = find(strcmp('pressao_diferencial_BCSS', variaveis_alvo));
        predicoes_tempo(ipressao_diferencial) = predicao;
    end


    tempo_execucao = toc(tStart)  ;
    predicoes_tempo(end)= tempo_execucao;

end


