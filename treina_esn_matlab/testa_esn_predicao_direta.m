function testa_esn_predicao_direta(esn, df, periodo_validacao, variaveis_preditoras, variaveis_preditas, variaveis_apelidos_unidades, plotar, normalizar_v2)
    if nargin < 8
           normalizar_v2 = 0;
    end

    if normalizar_v2==1
        disp('Normalizacao v2');
    end

    disp('Testando ESN com o período:');
    disp(periodo_validacao);
    n_out = length(variaveis_preditas);   % 10 (número de variáveis de saída)

    
    % Seleciona dados do período
    idx_periodo = df.data_hora >= periodo_validacao{1} & df.data_hora <= periodo_validacao{2};
    df_periodo = df(idx_periodo, :);
    matriz_df = table2array(df_periodo(:, variaveis_preditoras)); 
    %normalizando
    for k=1:length(matriz_df)
        if normalizar_v2==1
            matriz_df(k, :) = normaliza_entradas_v2(matriz_df(k, :));
        else
            matriz_df(k, :) = normaliza_entradas(matriz_df(k, :));
        end
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
        if normalizar_v2==1
            y_pred(k, :) = desnormaliza_predicoes_v2(y_pred(k,:));
        else
            y_pred(k, :) = desnormaliza_predicoes(y_pred(k,:));
        end

    end
    
    % Calcula erro de teste
    mse = mean((y_teste - y_pred).^2);
    mape = mean(abs((y_teste - y_pred) ./ y_teste)) * 100;
    
    fprintf('Erro de teste (MAPE) geral: %.2f%%\n', mean(mape));
    fprintf('Erro de teste (MSE) geral: %.6f\n', mean(mse));
    fprintf('Erro de teste (RMSE) geral: %.6f\n', mean(sqrt(mse)));
    
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

