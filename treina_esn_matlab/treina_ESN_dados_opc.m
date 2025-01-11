clear;
close all;


nome_arquivo_dados= 'df_opc_mpa_10s.parquet';

% Configuração da semente aleatória
rng(42);

% Constantes para experimentar no treinamento
taxa_vazamento = 0.9;
raio_espectral = 0.99;    
tamanho_do_reservatorio = 100;
reg_min = 1e-6;
reg_max = 1e-1;

cv_folds = 5;

plotar = 0;


% Definição das variáveis
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

% Definir as variáveis e suas unidades
variaveis_apelidos_unidades = {
    {0, 'P Succao', 'bar'}
    {1, 'P Chegada', 'bar'}
    {2, 'P Diferencial', 'bar'}
    {3, 'P Descarga', 'bar'}
    {4, 'T Motor', '°C'}
    {5, 'C Torque', 'A'}
    {6, 'C Total', 'A'}
    {7, 'T Succao', '°C'}
    {8, 'Vibracao', 'g'}
    {9, 'T Chegada', '°C'}
};



% Definir dimensões
n_in = length(variaveis_preditoras);  % 12 (número de variáveis de entrada)
n_out = length(variaveis_preditas);   % 10 (número de variáveis de saída)

% Definir períodos de interesse
periodos_interesse = {
    {'2024-07-12 09:30:00', '2024-07-12 12:30:00'}  % 1 - transiente pmon subindo
    {'2024-07-12 12:30:00', '2024-07-12 16:59:00'}  % 2 - transiente freqs e pmon descendo
    {'2024-07-12 23:06:04', '2024-07-13 02:41:00'}  % 3 - transiente pmon subindo
    {'2024-07-13 03:30:00', '2024-07-13 06:48:00'}  % 4 - transientes freq descendo
    {'2024-07-15 13:33:30', '2024-07-15 22:00:00'}  % 5 - transiente pmon sobe
    {'2024-07-16 22:00:00', '2024-07-17 05:00:00'}  % 6 - transiente pmon desce  
};

% Período de validação
periodo_validacao = {'2024-09-09 00:00:00', '2024-09-09 13:20:00'};


df=parquetread(nome_arquivo_dados);
df=fillmissing(df,'previous');


disp('Cria a ESN com parâmetros personalizados');

esn = ESN(tamanho_do_reservatorio, n_in, n_out, ...
    'gama', taxa_vazamento, ...        % taxa de vazamento
    'ro', raio_espectral, ...         % raio espectral
    'psi', 0.0, ...        % esparsidade
    'in_scale', 0.1, ...   % escala de entrada
    'noise_amplitude', 0, ...
    'bias_scale', 0.15); 




disp('Pré-processa dados para treinamento')
for i = 1:length(periodos_interesse)
    periodo = periodos_interesse{i};
    disp(periodo);
    
    % Seleciona dados do período
    idx_periodo = df.data_hora >= periodo{1} & df.data_hora <= periodo{2};
    df_periodo = df(idx_periodo, :);
    matriz_df = table2array(df_periodo(:, variaveis_preditoras)); 

    %normalizando
    for k=1:length(matriz_df)
        matriz_df(k, :) = normaliza_entradas(matriz_df(k,:));
    end
    
    y_treina = matriz_df(2:end, 3:end);
    
    u_treina = matriz_df(1:end-1, :);
    
    % Adiciona dados ao ESN
    esn.add_data(u_treina, y_treina);
end

disp('Treinando ESN...');
[erro_cv, melhor_reg] = esn.cum_train_cv(reg_min, reg_max);
fprintf('Melhor regularização: %.10f\n', melhor_reg);
fprintf('Erro CV: %.6f\n', erro_cv);

% Cria nome do modelo com os parâmetros
nome_arquivo_modelo = sprintf('weightsESNx_TR%d_TVaz%.2f_RaioE%.2f.mat', ...
    tamanho_do_reservatorio, taxa_vazamento, raio_espectral);
disp(nome_arquivo_modelo)

% Salva o modelo
esn.save_reservoir(nome_arquivo_modelo);

testa_esn(esn, df, periodo_validacao, variaveis_preditoras, variaveis_preditas, plotar);

%nome_arquivo_modelo_leizer = 'weightsESNx_TR400_TVaz0.9_RaioE0.4.mat';
%disp('Testando outra esn')
%disp(nome_arquivo_modelo_leizer)
%esn.load_reservoir(nome_arquivo_modelo_leizer);
%testa_esn(esn, df, periodo_validacao, variaveis_preditoras, variaveis_preditas, plotar);


function testa_esn(esn, df, periodo_validacao, variaveis_preditoras, variaveis_preditas, plotar)
    disp('Testando ESN com o período:');
    disp(periodo_validacao);
    
    % Seleciona dados do período
    idx_periodo = df.data_hora >= periodo_validacao{1} & df.data_hora <= periodo_validacao{2};
    df_periodo = df(idx_periodo, :);
    matriz_df = table2array(df_periodo(:, variaveis_preditoras)); 
    %normalizando
    for k=1:length(matriz_df)
        matriz_df(k, :) = normaliza_entradas(matriz_df(k,:));
    end
    
    u_teste = matriz_df(1:end-1, :);
    %y_teste = matriz_df(2:end, 3:end);
    y_teste = table2array(df_periodo(2:end, variaveis_preditas)); 
    y_pred = zeros(size(y_teste));
    
    %esquenta modelo para o ponto atual dos dados
    for i=1:1000
        esn.update(u_teste(1,:));
    end
    
    % Previsão
    for i = 1:length(u_teste)
        y_pred(i,:) = esn.update(u_teste(i,:));
    end
    
    %desnormaliza predicoes
    for k=1:length(y_pred)
        y_pred(k, :) = desnormaliza_predicoes(y_pred(k,:));
    end
    
    % Calcula erro de teste
    mse = mean((y_teste - y_pred).^2);
    mape = mean(abs((y_teste - y_pred) ./ y_teste)) * 100;
    fprintf('Erro de teste (MSE) geral: %.6f\n', mean(mse));
    fprintf('Erro de teste (RMSE) geral: %.6f\n', mean(sqrt(mse)));
    fprintf('Erro de teste (MAPE) geral: %.2f%%\n', mean(mape));
    for i=1:length(variaveis_preditas)
        mape = mean(abs((y_teste(:,i) - y_pred(:,i)) ./ y_teste(:,i))) * 100;
        %disp(variaveis_preditas{i})
        fprintf('Erro de teste (MAPE) %s: %.2f%%\n', variaveis_preditas{i}, mape);
    end    


    if plotar
    
        %% Plots dos resultados
        
        % Configurações gerais dos plots
        set(0,'defaultAxesFontSize',12)
        set(0,'defaultTextFontSize',12)
        set(0,'defaultAxesFontName','Arial')
        set(0,'defaultTextFontName','Arial')
        set(0,'defaultLineLineWidth',1.5)
        
        %% 1. Plot do período de teste - Todas as variáveis em subplots
        figure('Position', [100 100 1200 800])
        n_rows = ceil(n_out/2);  % 2 colunas
        for i = 1:n_out
            subplot(n_rows, 2, i)
            plot(df_periodo.data_hora(1:end-1), y_teste(:,i), 'b-', 'DisplayName', 'Real')
            hold on
            plot(df_periodo.data_hora(1:end-1), y_pred(:,i), 'r--', 'DisplayName', 'Previsto')
            title(variaveis_apelidos_unidades{i}{2})
            grid on
            legend('show')
            if i == n_out-1 || i == n_out  % Apenas últimos subplots
                xlabel('Tempo')
            end
            %ylabel('Valor Normalizado')
            ylabel(variaveis_apelidos_unidades{i}{3});
            hold off
        end
        sgtitle('Comparação entre Valores Reais e Previstos - Período de Teste')
        
        %% 2. Plot de dispersão - Previsto vs Real
        figure('Position', [100 100 1200 800])
        n_rows = ceil(n_out/2);
        for i = 1:n_out
            subplot(n_rows, 2, i)
            scatter(y_teste(:,i), y_pred(:,i), 20, 'filled', 'MarkerFaceAlpha', 0.5)
            hold on
            % Linha ideal (y=x)
            min_val = min(min(y_teste(:,i)), min(y_pred(:,i)));
            max_val = max(max(y_teste(:,i)), max(y_pred(:,i)));
            plot([min_val max_val], [min_val max_val], 'r--')
            title(variaveis_preditas{i})
            xlabel('Valor Real')
            ylabel('Valor Previsto')
            grid on
            hold off
        end
        sgtitle('Gráficos de Dispersão - Valores Previstos vs Reais')
        
        %% 3. Plot de erro ao longo do tempo
        erros = y_teste - y_pred;
        figure('Position', [100 100 1200 800])
        n_rows = ceil(n_out/2);
        for i = 1:n_out
            subplot(n_rows, 2, i)
            plot(df_periodo.data_hora(1:end-1), erros(:,i))
            title(['Erro - ', variaveis_preditas{i}])
            grid on
            if i == n_out-1 || i == n_out  % Apenas últimos subplots
                xlabel('Tempo')
            end
            ylabel('Erro')
        end
        sgtitle('Erro de Previsão ao Longo do Tempo')
    end
end

