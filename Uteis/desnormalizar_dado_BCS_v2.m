function dado_desnormalizado = desnormalizar_dado_BCS_v2(dado, coluna)
    %DESNORMALIZAR_DADO_BCS Desnormaliza os dados com base na coluna especificada.
    % 
    %   dado_desnormalizado = DESNORMALIZAR_DADO_BCS(dado, coluna) desnormaliza
    %   o dado fornecido com base nos valores mínimos e máximos da coluna especificada.
    %
    %   Entrada:
    %       dado - Valor ou array de valores a serem desnormalizados.
    %       coluna - Nome da coluna como uma string. Exemplo: 'frequencia_BCSS'.
    %
    %   Saída:
    %       dado_desnormalizado - Valor ou array de valores desnormalizados.
    %
    %   Exemplo de uso:
    %       dado_desnormalizado = desnormalizar_dado_BCS(0.5, 'frequencia_BCSS');
    
    [min_value, max_value] = get_min_max_BCSS_v2(coluna);
    dado_desnormalizado = (dado * (max_value - min_value)) + min_value;
end