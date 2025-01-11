function [min_value, max_value] = get_min_max_BCSS(coluna)
    %GET_MIN_MAX_BCSS Retorna os valores mínimos e máximos para normalização de colunas específicas.
    % 
    %   [min_value, max_value] = GET_MIN_MAX_BCSS(coluna) retorna os valores 
    %   mínimos e máximos para a normalização dos dados da coluna especificada.
    %
    %   Entrada:
    %       coluna - Nome da coluna como uma string. Exemplo: 'frequencia_BCSS'.
    %
    %   Saída:
    %       min_value - Valor mínimo da coluna para normalização.
    %       max_value - Valor máximo da coluna para normalização.
    %
    %   Exemplo de uso:
    %       [min_value, max_value] = get_min_max_BCSS('frequencia_BCSS');
    
    persistent min_max_values;
    
    if isempty(min_max_values)
        min_max_values = containers.Map;
        
        min_max_values('frequencia_BCSS') = [0, 60];
        min_max_values('choke_producao') = [0, 100];
        min_max_values('pressao_chegada') = [0, 100];
        min_max_values('pressao_succao_BCSS') = [0, 250];
        min_max_values('pressao_diferencial_BCSS') = [0, 200];
        min_max_values('pressao_descarga_BCSS') = [0, 300];
        min_max_values('PDG_pressao') = [0, 300];
        min_max_values('temperatura_succao_BCSS') = [0, 250];
        min_max_values('temperatura_motor_BCSS') = [0, 200];
        min_max_values('PDG_temperatura') = [0, 300];
        min_max_values('vibracao_BCSS') = [0, 100];
        min_max_values('temperatura_chegada') = [0, 100];
        min_max_values('header_psi_201a') = [0, 100];
        min_max_values('corrente_total_BCSS') = [0, 200];
        min_max_values('corrente_torque_BCSS') = [0, 200];
        min_max_values('tensao_saida_VSD') = [0, 5000];
        min_max_values('pressao_montante_alvo') = [0, 100];
    end
    
    if isKey(min_max_values, coluna)
        values = min_max_values(coluna);
        min_value = values(1);
        max_value = values(2);
    else
        error('Coluna desconhecida, não é possivel normalizar.');
    end
end

