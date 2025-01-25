clear;
close all;

plotar = 0;

horizonte_predicao = 100; % Horizonte de predição definido

modelos = {
    'weightsESNx_TR100_TVaz0.50_RaioE0.50_mape_msa_dois_2.60.mat'
    'weightsESNx_TR300_TVaz0.8_RaioE0.1.mat'
    'weightsESNx_TR400_TVaz0.9_RaioE0.4.mat'
    'weightsESNx_TR900_TVaz0.9_RaioE0.4.mat'
    };
nome_arquivo_dados = 'df_opc_mpa_10s.parquet';
%periodo_validacao = {'2024-09-09 00:00:00', '2024-09-09 13:20:00'};
periodo_validacao = {'2024-07-12 23:06:04', '2024-07-13 06:48:00'};

% Definição das variáveis (mantidas do código original)
variaveis_manipuladas = {
    'frequencia_BCSS'
    'pressao_montante_alvo'
};

variaveis_preditas = {
    'pressao_succao_BCSS'
    'pressao_chegada'
    'pressao_diferencial_BCSS'
    'pressao_descarga_BCSS'
    'temperatura_motor_BCSS'
    'corrente_torque_BCSS'
    'corrente_total_BCSS'
    'temperatura_succao_BCSS'
    'vibracao_BCSS'
    'temperatura_chegada'
};

% Concatenar variáveis preditoras
variaveis_preditoras = [variaveis_manipuladas; variaveis_preditas];

% Definir dimensões
n_in = length(variaveis_preditoras);
n_out = length(variaveis_preditas);

% Carregar dados
df = parquetread(nome_arquivo_dados);
df = fillmissing(df, 'previous');

% Seleciona dados do período
idx_periodo = df.data_hora >= periodo_validacao{1} & df.data_hora <= periodo_validacao{2};
df_periodo = df(idx_periodo, :);


tempos = zeros(length(modelos), 1);

for m = 1:length(modelos)
    nome_arquivo_modelo = modelos{m};
    % Criar e carregar ESN
    esn = ESN(100, n_in, n_out);
    disp(['Carregando modelo: ' nome_arquivo_modelo]);
    esn.load_reservoir(nome_arquivo_modelo);
    
    % Executar medição de tempo
    disp(['Medindo tempo para ' num2str(horizonte_predicao) ' predições...']);
    tempos(m) = mede_tempo_predicoes(esn, df, variaveis_preditoras, variaveis_manipuladas, horizonte_predicao, plotar);
    plot(tempos)

end


% Função para medir tempo de predições
function tempo_total = mede_tempo_predicoes(esn, df, variaveis_preditoras, variaveis_manipuladas, horizonte_predicao, plotar)
    n_in = length(variaveis_preditoras);
    n_manipuladas = length(variaveis_manipuladas);
    
    % Preparar dados iniciais
    matriz_df = table2array(df(1:horizonte_predicao+1, variaveis_preditoras));
    matriz_df_norm = zeros(size(matriz_df));
    
    % Normalizar dados
    for k = 1:size(matriz_df, 1)
        matriz_df_norm(k, :) = normaliza_entradas(matriz_df(k, :));
    end
    
    % Inicializar estado
    estado_atual = matriz_df_norm(1, :);
    
    % Aquecer o reservatório
    disp('Aquecendo o reservatório...');
    tic;
    for w = 1:1000
        esn.update(estado_atual);
    end
    tempo_aquecimento = toc;
    disp(['Tempo de aquecimento: ' num2str(tempo_aquecimento) ' segundos']);
    
    % Realizar predições e medir tempo
    disp('Iniciando predições...');
    tempos = zeros(horizonte_predicao, 1);
    
    for i = 1:horizonte_predicao
        tic;
        % Fazer predição
        pred_norm = esn.update(estado_atual);
        
        % Preparar próximo estado
        estado_atual = zeros(1, n_in);
        if i < horizonte_predicao
            estado_atual(1:n_manipuladas) = matriz_df_norm(i+1, 1:n_manipuladas);
        end
        estado_atual(n_manipuladas+1:end) = pred_norm;
        
        % Registrar tempo
        tempos(i) = toc;
        disp(['Predição ' num2str(i) ': ' num2str(tempos(i)) ' segundos']);
    end
    if(plotar)
        plot(tempos)
    end
    % Mostrar resultados
    tempo_total = sum(tempos);
    tempo_medio = mean(tempos);
    disp(['Tempo total de predição: ' num2str(tempo_total) ' segundos']);
    disp(['Tempo médio por predição: ' num2str(tempo_medio) ' segundos']);
end

