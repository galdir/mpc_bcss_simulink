import numpy as np

def normaliza_entradas(u):
    """Normaliza as entradas."""
    uk = np.array([
        normalizar_dado_BCS(u[0], 'frequencia_BCSS'),
        normalizar_dado_BCS(u[1], 'pressao_montante_alvo'),
        normalizar_dado_BCS(u[2], 'pressao_succao_BCSS'),
        normalizar_dado_BCS(u[3], 'pressao_chegada'),
        normalizar_dado_BCS(u[4], 'pressao_diferencial_BCSS'),
        normalizar_dado_BCS(u[5], 'pressao_descarga_BCSS'),
        normalizar_dado_BCS(u[6], 'temperatura_motor_BCSS'),
        normalizar_dado_BCS(u[7], 'corrente_torque_BCSS'),
        normalizar_dado_BCS(u[8], 'corrente_total_BCSS'),
        normalizar_dado_BCS(u[9], 'temperatura_succao_BCSS'),
        normalizar_dado_BCS(u[10], 'vibracao_BCSS'),
        normalizar_dado_BCS(u[11], 'temperatura_chegada')
    ])
    return uk

def desnormaliza_predicoes(yk_aux):
    """Desnormaliza as saídas."""
    yk_1 = np.array([
        desnormalizar_dado_BCS(yk_aux[0], 'pressao_succao_BCSS'),
        desnormalizar_dado_BCS(yk_aux[1], 'pressao_chegada'),
        desnormalizar_dado_BCS(yk_aux[2], 'pressao_diferencial_BCSS'),
        desnormalizar_dado_BCS(yk_aux[3], 'pressao_descarga_BCSS'),
        desnormalizar_dado_BCS(yk_aux[4], 'temperatura_motor_BCSS'),
        desnormalizar_dado_BCS(yk_aux[5], 'corrente_torque_BCSS'),
        desnormalizar_dado_BCS(yk_aux[6], 'corrente_total_BCSS'),
        desnormalizar_dado_BCS(yk_aux[7], 'temperatura_succao_BCSS'),
        desnormalizar_dado_BCS(yk_aux[8], 'vibracao_BCSS'),
        desnormalizar_dado_BCS(yk_aux[9], 'temperatura_chegada')
    ])
    return yk_1

def normalizar_dado_BCS(dados, coluna):
    """
    Normaliza os dados com base na coluna especificada.
    
    Args:
        dados: Valor ou array de valores a serem normalizados.
        coluna: Nome da coluna como uma string. Exemplo: 'frequencia_BCSS'.
    
    Returns:
        dados_normalizados: Valor ou array de valores normalizados.
    
    Exemplo de uso:
        dado_normalizado = normalizar_dado_BCS(30, 'frequencia_BCSS')
    """
    min_value, max_value = get_min_max_BCSS(coluna)
    dados_normalizados = (dados - min_value) / (max_value - min_value)
    return dados_normalizados

def get_min_max_BCSS(coluna):
    """
    Retorna os valores mínimos e máximos para normalização de colunas específicas.
    
    Args:
        coluna: Nome da coluna como uma string. Exemplo: 'frequencia_BCSS'.
    
    Returns:
        min_value: Valor mínimo da coluna para normalização.
        max_value: Valor máximo da coluna para normalização.
    
    Exemplo de uso:
        min_value, max_value = get_min_max_BCSS('frequencia_BCSS')
    """
    min_max_values = {
        'frequencia_BCSS': (0, 60),
        'choke_producao': (0, 100),
        'pressao_chegada': (0, 100),
        'pressao_succao_BCSS': (0, 250),
        'pressao_diferencial_BCSS': (0, 200),
        'pressao_descarga_BCSS': (0, 300),
        'PDG_pressao': (0, 300),
        'temperatura_succao_BCSS': (0, 250),
        'temperatura_motor_BCSS': (0, 200),
        'PDG_temperatura': (0, 300),
        'vibracao_BCSS': (0, 100),
        'temperatura_chegada': (0, 100),
        'header_psi_201a': (0, 100),
        'corrente_total_BCSS': (0, 200),
        'corrente_torque_BCSS': (0, 200),
        'tensao_saida_VSD': (0, 5000),
        'pressao_montante_alvo': (0, 100)
    }
    
    if coluna in min_max_values:
        return min_max_values[coluna]
    else:
        raise ValueError('Coluna desconhecida, não é possível normalizar.')

def desnormalizar_dado_BCS(dado, coluna):
    """
    Desnormaliza os dados com base na coluna especificada.
    
    Args:
        dado: Valor ou array de valores a serem desnormalizados.
        coluna: Nome da coluna como uma string. Exemplo: 'frequencia_BCSS'.
    
    Returns:
        dado_desnormalizado: Valor ou array de valores desnormalizados.
    
    Exemplo de uso:
        dado_desnormalizado = desnormalizar_dado_BCS(0.5, 'frequencia_BCSS')
    """
    min_value, max_value = get_min_max_BCSS(coluna)
    dado_desnormalizado = (dado * (max_value - min_value)) + min_value
    return dado_desnormalizado