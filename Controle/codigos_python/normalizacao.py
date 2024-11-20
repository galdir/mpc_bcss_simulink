def get_min_max_BCSS(coluna: str) -> tuple[float, float]:
    """
    Retorna os valores mínimos e máximos para normalização de colunas específicas.

    Parâmetros:
        coluna (str): Nome da coluna. Exemplo: 'frequencia_BCSS'

    Retorna:
        tuple[float, float]: Uma tupla contendo (valor_minimo, valor_maximo)

    Exemplo de uso:
        min_value, max_value = get_min_max_BCSS('frequencia_BCSS')

    Raises:
        KeyError: Se a coluna especificada não existir no dicionário de valores
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

    try:
        min_value, max_value = min_max_values[coluna]
        return min_value, max_value
    except KeyError:
        raise KeyError(
            f'Coluna desconhecida: "{coluna}", não é possível normalizar.')


def normalizar_dado_BCS(dados, coluna: str):
    """
    Normaliza os dados com base na coluna especificada.

    Parâmetros:
        dados: Valor ou array de valores a serem normalizados
        coluna (str): Nome da coluna. Exemplo: 'frequencia_BCSS'

    Retorna:
        Valor ou array de valores normalizados no intervalo [0,1]

    Exemplo de uso:
        dado_normalizado = normalizar_dado_BCS(30, 'frequencia_BCSS')

    Raises:
        KeyError: Se a coluna especificada não existir
    """
    min_value, max_value = get_min_max_BCSS(coluna)
    dados_normalizados = (dados - min_value) / (max_value - min_value)
    return dados_normalizados


def normaliza_entradas(u):
    """
    Normaliza as entradas do sistema BCS.

    Parâmetros:
        u: Array com as entradas do sistema. Deve conter 12 valores na seguinte ordem:
           [frequencia, pressao_montante, pressao_succao, pressao_chegada, 
            pressao_diferencial, pressao_descarga, temperatura_motor, corrente_torque,
            corrente_total, temperatura_succao, vibracao, temperatura_chegada]

    Retorna:
        Array com todas as entradas normalizadas na mesma ordem

    Raises:
        IndexError: Se o array de entrada não tiver o tamanho correto
        KeyError: Se alguma coluna não existir no sistema de normalização
    """
    # Normaliza as entradas principais
    frequencia_BCSS = normalizar_dado_BCS(u[0], 'frequencia_BCSS')
    pressao_montante_alvo = normalizar_dado_BCS(u[1], 'pressao_montante_alvo')

    # Normaliza os regressores
    pressao_succao_BCSS = normalizar_dado_BCS(u[2], 'pressao_succao_BCSS')
    pressao_chegada = normalizar_dado_BCS(u[3], 'pressao_chegada')
    pressao_diferencial_BCSS = normalizar_dado_BCS(u[4], 'pressao_diferencial_BCSS')
    pressao_descarga_BCSS = normalizar_dado_BCS(u[5], 'pressao_descarga_BCSS')
    temperatura_motor_BCSS = normalizar_dado_BCS(u[6], 'temperatura_motor_BCSS')
    corrente_torque_BCSS = normalizar_dado_BCS(u[7], 'corrente_torque_BCSS')
    corrente_total_BCSS = normalizar_dado_BCS(u[8], 'corrente_total_BCSS')
    temperatura_succao_BCSS = normalizar_dado_BCS(u[9], 'temperatura_succao_BCSS')
    vibracao_BCSS = normalizar_dado_BCS(u[10], 'vibracao_BCSS')
    temperatura_chegada = normalizar_dado_BCS(u[11], 'temperatura_chegada')

    # Retorna array com todos os valores normalizados
    # import numpy as np

    # return np.array([
    #     freq_n, Pmon_n, Psuc_n, Pche_n, Pdifer_n, Pdesc_n,
    #     Tmotor_n, Ctorque_n, Ctotal_n, Tsuc_n, Vib_n, Tche_n
    # ])
    resultado = [
        frequencia_BCSS, pressao_montante_alvo, pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS,
        temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
    ]
    return resultado


def desnormalizar_dado_BCS(dado, coluna: str):
    """
    Desnormaliza os dados com base na coluna especificada.

    Parâmetros:
        dado: Valor ou array de valores normalizados a serem desnormalizados
        coluna (str): Nome da coluna. Exemplo: 'frequencia_BCSS'

    Retorna:
        Valor ou array de valores desnormalizados nas unidades originais

    Exemplo de uso:
        dado_desnormalizado = desnormalizar_dado_BCS(0.5, 'frequencia_BCSS')

    Raises:
        KeyError: Se a coluna especificada não existir
    """
    min_value, max_value = get_min_max_BCSS(coluna)
    dado_desnormalizado = (dado * (max_value - min_value)) + min_value
    return dado_desnormalizado


def desnormaliza_predicoes(yk_aux):
    """
    Desnormaliza as predições do sistema BCS.

    Parâmetros:
        yk_aux: Array com as predições normalizadas. Deve conter 10 valores na seguinte ordem:
               [pressao_succao, pressao_chegada, pressao_diferencial, pressao_descarga,
                temperatura_motor, corrente_torque, corrente_total, temperatura_succao,
                vibracao, temperatura_chegada]

    Retorna:
        Array numpy com todos os valores desnormalizados na mesma ordem

    Raises:
        IndexError: Se o array de entrada não tiver o tamanho correto
        KeyError: Se alguma coluna não existir no sistema de normalização
    """
    # Desnormaliza cada uma das saídas
    pressao_succao_BCSS = desnormalizar_dado_BCS(
        yk_aux[0], 'pressao_succao_BCSS')
    pressao_chegada = desnormalizar_dado_BCS(yk_aux[1], 'pressao_chegada')
    pressao_diferencial_BCSS = desnormalizar_dado_BCS(
        yk_aux[2], 'pressao_diferencial_BCSS')
    pressao_descarga_BCSS = desnormalizar_dado_BCS(
        yk_aux[3], 'pressao_descarga_BCSS')
    temperatura_motor_BCSS = desnormalizar_dado_BCS(
        yk_aux[4], 'temperatura_motor_BCSS')
    corrente_torque_BCSS = desnormalizar_dado_BCS(
        yk_aux[5], 'corrente_torque_BCSS')
    corrente_total_BCSS = desnormalizar_dado_BCS(
        yk_aux[6], 'corrente_total_BCSS')
    temperatura_succao_BCSS = desnormalizar_dado_BCS(
        yk_aux[7], 'temperatura_succao_BCSS')
    vibracao_BCSS = desnormalizar_dado_BCS(yk_aux[8], 'vibracao_BCSS')
    temperatura_chegada = desnormalizar_dado_BCS(
        yk_aux[9], 'temperatura_chegada')

    # Retorna array numpy com todos os valores desnormalizados
    # import numpy as np
    # return np.array([
    #     pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, temperatura_motor_BCSS,
    #     corrente_torque_BCSS, corrente_total_BCSS, temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
    # ])

    return [
        pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, temperatura_motor_BCSS,
        corrente_torque_BCSS, corrente_total_BCSS, temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
    ]
