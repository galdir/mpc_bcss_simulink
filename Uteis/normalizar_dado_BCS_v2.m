function dados_normalizados = normalizar_dado_BCS_v2(dados, coluna)
    %NORMALIZAR_DADO_BCS Normaliza os dados com base na coluna especificada.
    % 
    %   dado_normalizado = NORMALIZAR_DADO_BCS(dado, coluna) normaliza o dado
    %   fornecido com base nos valores mínimos e máximos da coluna especificada.
    %
    %   Entrada:
    %       dado - Valor ou array de valores a serem normalizados.
    %       coluna - Nome da coluna como uma string. Exemplo: 'frequencia_BCSS'.
    %
    %   Saída:
    %       dado_normalizado - Valor ou array de valores normalizados.
    %
    %   Exemplo de uso:
    %       dado_normalizado = normalizar_dado_BCS(30, 'frequencia_BCSS');

    [min_value, max_value] = get_min_max_BCSS_v2(coluna);
    dados_normalizados = (dados - min_value) / (max_value - min_value);
end


