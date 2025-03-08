function mape_geral = testa_esn_multi_passos(esn, df, periodo_validacao, variaveis_preditoras, variaveis_preditas, variaveis_manipuladas, variaveis_apelidos_unidades, plotar, normalizar_v2)
    
    if nargin < 9
           normalizar_v2 = 0;
    end

    if normalizar_v2==1
        disp('Normalizacao v2');
    end
    
    disp(' ');
    disp('Testando ESN com previsão multi-passos para o período:');
    disp(periodo_validacao);
    n_in = length(variaveis_preditoras);
    n_out = length(variaveis_preditas);
    
    % Seleciona dados do período
    idx_periodo = df.data_hora >= periodo_validacao{1} & df.data_hora <= periodo_validacao{2};
    df_periodo = df(idx_periodo, :);
    
    % Preparar dados
    matriz_df = table2array(df_periodo(:, variaveis_preditoras));
    
    % Dados reais para comparação
    y_real = table2array(df_periodo(:, variaveis_preditas));
    
    % Número total de passos para prever
    n_steps = size(matriz_df, 1);
    n_manipuladas = length(variaveis_manipuladas);
    
    % Normalizar todo o conjunto de dados uma vez
    matriz_df_norm = zeros(size(matriz_df));
    for k = 1:size(matriz_df, 1)
        if normalizar_v2==1
            matriz_df_norm(k, :) = normaliza_entradas_v2(matriz_df(k, :));
        else
            matriz_df_norm(k, :) = normaliza_entradas(matriz_df(k, :));
        end
    end
    
    % Inicializar estado com o primeiro ponto e aquecer o reservatório
    estado_atual = matriz_df_norm(1, :);

    for w = 1:1000
        esn.update(estado_atual);
    end
    
    % Inicializar matriz de previsões
    y_pred = zeros(n_steps, length(variaveis_preditas));
    estados = zeros(n_steps, length(variaveis_preditoras));
    estados(1, :) = estado_atual;
    
    % Primeira previsão
    pred_norm = esn.update(estado_atual);
    if normalizar_v2==1
        pred = desnormaliza_predicoes_v2(pred_norm);
    else
        pred = desnormaliza_predicoes(pred_norm);
    end

    y_pred(1, :) = pred;
    
    % Fazer previsões multi-passos
    for i = 2:n_steps
        % Criar novo estado:
        % - Usar variáveis manipuladas reais do próximo passo
        % - Usar previsões normalizadas para as variáveis preditas
        estado_atual = zeros(1, n_in);
        
        % Copiar variáveis manipuladas do próximo passo
        estado_atual(1:n_manipuladas) = matriz_df_norm(i, 1:n_manipuladas);
        
        % Usar a última previsão normalizada para as variáveis preditas
        estado_atual(n_manipuladas+1:end) = pred_norm;
        
        % Salvar estado
        estados(i, :) = estado_atual;
        
        % Fazer nova previsão
        pred_norm = esn.update(estado_atual);
        if normalizar_v2==1
            pred = desnormaliza_predicoes_v2(pred_norm);
        else
            pred = desnormaliza_predicoes(pred_norm);
        end

        y_pred(i, :) = pred;
    end
    
    %mape = mean(abs((y_real(:,v) - y_pred(:,v)) ./ y_real(:,v))) * 100;
    mape_geral = mean(abs((y_real - y_pred) ./ y_real)) * 100;
    fprintf('MAPE geral: %.2f%%\n', mean(mape_geral));
        
    % Calcular erros
    for v = 1:length(variaveis_preditas)
        mape = mean(abs((y_real(:,v) - y_pred(:,v)) ./ y_real(:,v))) * 100;
        fprintf('%s - MAPE: %.2f%%\n', variaveis_preditas{v}, mape);
        
        % Prints de debug
%         if v == 1
%             fprintf('\nPrimeiros valores para %s:\n', variaveis_preditas{v});
%             fprintf('Real: %.4f, Previsto: %.4f\n', y_real(1,v), y_pred(1,v));
%             fprintf('Real: %.4f, Previsto: %.4f\n', y_real(2,v), y_pred(2,v));
%             fprintf('Real: %.4f, Previsto: %.4f\n', y_real(3,v), y_pred(3,v));
%         end
    end
    
    if plotar
        % Plotar resultados
        plot_results(df_periodo.data_hora, y_pred, y_real, variaveis_preditas, variaveis_apelidos_unidades);
    end
    
    % Salvar estados e previsões para debug se necessário
    save('debug_dados.mat', 'estados', 'y_pred', 'y_real', 'matriz_df_norm');

end 

function plot_results(tempo, y_pred, y_real, variaveis_preditas, variaveis_apelidos_unidades)
    % Configurações dos plots
    set(0, 'defaultAxesFontSize', 12)
    set(0, 'defaultTextFontSize', 12)
    set(0, 'defaultAxesFontName', 'Arial')
    set(0, 'defaultTextFontName', 'Arial')
    set(0, 'defaultLineLineWidth', 1.5)
    
    % Plot para cada variável
    n_out = length(variaveis_preditas);
    figure('Position', [100 100 1200 800])
    n_rows = ceil(n_out/2);
    
    for i = 1:n_out
        subplot(n_rows, 2, i)
        plot(tempo, y_real(:,i), 'b-', 'DisplayName', 'Real')
        hold on
        plot(tempo, y_pred(:,i), 'r--', 'DisplayName', 'Previsão')
        
        title(variaveis_apelidos_unidades{i}{2})
        xlabel('Tempo')
        ylabel(variaveis_apelidos_unidades{i}{3})
        grid on
        legend('show')
        hold off
    end
    sgtitle('Previsões Multi-passos vs Valores Reais')
end
