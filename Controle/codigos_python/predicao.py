import numpy as np

from estima_vazao import cria_estimador_vazao_casadi
from normalizacao import desnormaliza_predicoes, normaliza_entradas
import casadi as ca


def executa_predicao(entrada_u, estado_x, ESNdataa0, modelo_ESN, estimador_vazao_casadi):
    """
    Executa a predição combinando ESN e tabela de vazão

    Parâmetros:
    EntradaU -- entradas do processo (frequencia_aplicada, pressao_montante_alvo_aplicada)
    EstadosX -- 11 variáveis do processo no instante atual
    ESNdataa0 -- estado atual do reservatório
    modelo_ESN -- modelo contendo Wrr, Wir, Wbr
    f_matrizVazao_sym -- função que retorna vazão estimada

    Retorna:
    predicoes -- vetor com predições dos novos estados
    novo_a0 -- novo estado do reservatório
    """
    conversao_bar_kgf = 1.019716

    # Executa predição da ESN
    predicoes, novo_a0 = executa_predicao_esn(
        entrada_u, estado_x, ESNdataa0, modelo_ESN)

    # Executa predição da vazão
    freq = entrada_u[0]
    # Converte pressão de bar para Kgf/cm2
    pressa_chegada = predicoes[1] * conversao_bar_kgf
    vazao_oleo_estimada = estimador_vazao_casadi(freq, pressa_chegada)

    # Monta vetor final incluindo a vazão estimada
    predicoes = np.append(predicoes, vazao_oleo_estimada)

    return predicoes, novo_a0


def executa_predicao_esn(entrada_u, entrada_x, estado_reservatorio, modelo_ESN):
    """
    Executa a predição da ESN um passo a frente

    Parâmetros:
    EntradaU -- entradas do processo no instante atual
    EstadosX -- variáveis do processo no instante atual
    ESNdataa0 -- estado atual do reservatório da ESN
    modelo_ESN -- modelo contendo Wrr, Wir, Wbr

    Retorna:
    predicoes -- vetor com estimativas das variáveis do processo
    novo_a0 -- novos estados do reservatório da ESN
    """
    if (len(estado_reservatorio.shape) > 0):
        Exception('estado do reservatorio precisa ter apenas uma dimensao')

    # Monta vetor de entrada (excluindo vazão)
    entradas = np.concatenate([entrada_u, entrada_x[:-1]])

    # Normaliza entradas
    entradas_normalizadas = normaliza_entradas(entradas)
    # entradas_normalizadas = entradas_normalizadas.reshape(-1)

    pesos_reservatorio = modelo_ESN['data']['Wrr'][0][0]
    pesos_entrada = modelo_ESN['data']['Wir'][0][0]
    pesos_bias_reservatorio = modelo_ESN['data']['Wbr'][0][0].reshape(-1)
    pesos_saida = modelo_ESN['data']['Wro'][0][0]

    leakrate = modelo_ESN['data']['gama'][0][0][0][0]

    # Calcula novo estado do reservatório
    z = (pesos_reservatorio @ estado_reservatorio +
         pesos_entrada @ entradas_normalizadas +
         pesos_bias_reservatorio)
    # z = (np.dot(pesos_reservatorio, estado_reservatorio) +
    #           np.dot(pesos_entrada, entradas_normalizadas) +
    #           pesos_bias_reservatorio)

    # Atualiza estado da ESN
    estado_reservatorio_novo = ((1 - leakrate) * estado_reservatorio +
                                leakrate * np.tanh(z))

    # Adiciona bias
    a_wbias = np.concatenate([[1.0], estado_reservatorio_novo])

    # Calcula predições
    predicoes_normalizadas = pesos_saida @ a_wbias

    # Desnormaliza as predições
    predicoes = desnormaliza_predicoes(predicoes_normalizadas)

    return predicoes, estado_reservatorio_novo


def executa_predicao_casadi(entradas_u, entradas_x, ESNdataa0, modelo_ESN, estimador_vazao_casadi):
    """
    Executa a predição combinando ESN e tabela de vazão

    Parâmetros:
    EntradaU -- entradas do processo (frequencia_aplicada, pressao_montante_alvo_aplicada)
    EstadosX -- 11 variáveis do processo no instante atual
    ESNdataa0 -- estado atual do reservatório
    modelo_ESN -- modelo contendo Wrr, Wir, Wbr
    f_matrizVazao_sym -- função que retorna vazão estimada

    Retorna:
    predicoes -- vetor com predições dos novos estados
    novo_a0 -- novo estado do reservatório
    """
    conversao_bar_kgf = 1.019716

    # Executa predição da ESN
    predicoes, novo_a0 = executa_predicao_esn_casadi(
        entradas_u, entradas_x, ESNdataa0, modelo_ESN)

    # Executa predição da vazão
    freq = entradas_u[0]
    # Converte pressão de bar para Kgf/cm2
    pressao_chegada_kgf = predicoes[1] * conversao_bar_kgf
    vazao_oleo_estimada = estimador_vazao_casadi(freq, pressao_chegada_kgf)

    # Monta vetor final incluindo a vazão estimada
    # predicoes = np.append(predicoes, vazao_oleo_estimada)
    # predicoes = ca.vertcat(predicoes, vazao_oleo_estimada)
    # predicoes_list = [predicoes[i] for i in range(predicoes.shape[0])]
    predicoes = ca.vertcat(*predicoes, vazao_oleo_estimada[0])

    return predicoes, novo_a0


def executa_predicao_esn_casadi(entrada_u, entrada_x, estado_reservatorio, modelo_ESN):
    """
    Executa a predição da ESN um passo a frente

    Parâmetros:
    EntradaU -- entradas do processo no instante atual
    EstadosX -- variáveis do processo no instante atual
    ESNdataa0 -- estado atual do reservatório da ESN
    modelo_ESN -- modelo contendo Wrr, Wir, Wbr

    Retorna:
    predicoes -- vetor com estimativas das variáveis do processo
    novo_a0 -- novos estados do reservatório da ESN
    """
    #if (len(estado_reservatorio.shape) > 0):
    #    Exception('estado do reservatorio precisa ter apenas uma dimensao')

    # estado_reservatorio = ca.MX(estado_reservatorio)
    # Monta vetor de entrada (excluindo vazão)
    # entradas = np.concatenate([entrada_u, entrada_x[:-1]])
    #entradas = ca.vertcat(entrada_u, entrada_x[:-1])
    entradas = ca.horzcat(entrada_u, entrada_x[:-1])

    # Normaliza entradas
    entradas_normalizadas = normaliza_entradas(entradas)
    entradas_normalizadas = ca.vertcat(*entradas_normalizadas)
    # entradas_normalizadas = ca.MX(entradas_normalizadas.reshape(-1))

    pesos_reservatorio = ca.MX(modelo_ESN['data']['Wrr'][0][0])
    pesos_entrada = ca.MX(modelo_ESN['data']['Wir'][0][0])
    pesos_bias_reservatorio = ca.MX(
        modelo_ESN['data']['Wbr'][0][0].reshape(-1))
    pesos_saida = ca.MX(modelo_ESN['data']['Wro'][0][0])
    # estado_reservatorio = modelo_ESN['data']['a0'][0][0].reshape(-1)
    # estado_reservatorio = estado_reservatorio.reshape(-1)
    leakrate = modelo_ESN['data']['gama'][0][0][0][0]

    # Calcula novo estado do reservatório
    # z = (pesos_reservatorio @ estado_reservatorio +
    #           pesos_entrada @ entradas_normalizadas +
    #           pesos_bias_reservatorio)
    # z = (np.dot(pesos_reservatorio, estado_reservatorio) +
    #           np.dot(pesos_entrada, entradas_normalizadas) +
    #           pesos_bias_reservatorio)
    z = ca.mtimes(pesos_reservatorio, estado_reservatorio) + \
        ca.mtimes(pesos_entrada, entradas_normalizadas) + \
        pesos_bias_reservatorio

    # Atualiza estado da ESN
    # estado_reservatorio_novo = ((1 - modelo_ESN['data']['gama'][0][0]) * estado_reservatorio +
    #            modelo_ESN['data']['gama'][0][0] * np.tanh(estado_reservatorio))
    estado_reservatorio_novo = ((1 - leakrate) * estado_reservatorio +
                                leakrate * ca.tanh(z))

    # Adiciona bias
    # a_wbias = np.concatenate([[1.0], estado_reservatorio_novo])
    a_wbias = ca.vertcat(1.0, estado_reservatorio_novo)

    # Calcula predições
    # predicoes_normalizadas = modelo_ESN['data']['Wro'][0][0] @ a_wbias
    predicoes_normalizadas = pesos_saida @ a_wbias

    # Desnormaliza as predições
    predicoes = desnormaliza_predicoes(predicoes_normalizadas)

    return predicoes, estado_reservatorio_novo


def esquentar_esn(entradas, reservatorio_esn, modelo_preditor, estimador_vazao):
    for k in range(1000):
        predicao, reservatorio_esn = executa_predicao(entradas[:2],
                                                      entradas[2:],
                                                      reservatorio_esn,
                                                      modelo_preditor,
                                                      estimador_vazao)
        
    return reservatorio_esn


# Exemplo de uso
if __name__ == "__main__":
    import scipy.io as sio
    from pathlib import Path
    import pandas as pd
    import casadi as ca

    conversao_bar_kgf = 1.019716

    # Nome do arquivo .mat a ser carregado
    nome_esn = 'weightsESNx_TR400_TVaz0.9_RaioE0.4.mat'
    caminho_modelos = Path("./Modelos/ESN/")
    caminho_modelo_esn = caminho_modelos / nome_esn

    arquivo_matriz_vazao = 'DoSimulador.xlsx'
    caminho_tabelas = Path("./Tabelas")
    df_matriz_vazao = pd.read_excel(
        caminho_tabelas / arquivo_matriz_vazao, skiprows=2, header=None)
    matriz = df_matriz_vazao.iloc[:, :3].values
    # Carrega o arquivo .mat
    modelo_preditor = sio.loadmat(caminho_modelo_esn)

    estimador_vazao = cria_estimador_vazao_casadi(matriz)

    pasta_dados = Path(
        'C:\\petrobras_2023_sistema_controle_inteligente_operacao_BCS_campo\\DADOS UTEIS\\dados MPA OPC\\')
    nome_arquivo_dados = 'df_opc_mpa_10s.parquet'

    df = pd.read_parquet(pasta_dados / nome_arquivo_dados)

    periodo = {'inicio': '2024-07-17 01:00:00',
               'fim': '2024-07-17 03:00:00'}

    predicao_matlab_k0 = [88.6705552895,
                          34.3950864586,
                          96.1270226239,
                          184.7975779136,
                          109.7271982542,
                          97.2411879156,
                          137.9484935089,
                          72.2509765211,
                          0.5282321748,
                          67.8458677256,
                          348.5237743013]

    df_periodo = df.query(
        'index >= @periodo["inicio"] and index <= @periodo["fim"]').copy()

    variaveis_manipuladas = ['frequencia_BCSS',
                             'pressao_montante_alvo']

    variaveis_processo = ['pressao_succao_BCSS',
                          'pressao_chegada',
                          'pressao_diferencial_BCSS',
                          'pressao_descarga_BCSS',
                          'temperatura_motor_BCSS',
                          'corrente_torque_BCSS',
                          'corrente_total_BCSS',
                          'temperatura_succao_BCSS',
                          'vibracao_BCSS',
                          'temperatura_chegada',
                          'vazao_oleo']

    # estima vazao inicial
    vazao = float(estimador_vazao(df_periodo['frequencia_BCSS'].iloc[0],
                                  df_periodo['pressao_chegada'].iloc[0]*conversao_bar_kgf))
    df_periodo['vazao_oleo'] = np.NaN
    df_periodo['vazao_oleo'].iloc[0] = vazao

    variaveis_preditoras = variaveis_manipuladas + variaveis_processo

    df_periodo_preditoras = df_periodo[variaveis_preditoras]

    reservatorio_esn = modelo_preditor['data']['a0'][0][0].reshape(-1)

    # esquentar ESN
    entradas = df_periodo_preditoras.iloc[0].values

    reservatorio_esn = esquentar_esn(entradas, reservatorio_esn, modelo_preditor, estimador_vazao)

    # for k in range(1000):
    #     predicao, reservatorio_esn = executa_predicao(entradas[:2],
    #                                                   entradas[2:],
    #                                                   reservatorio_esn,
    #                                                   modelo_preditor,
    #                                                   estimador_vazao)

    # entradas = df_periodo_preditoras.iloc[0].values

    predicao, reservatorio_esn = executa_predicao(entradas[:2],
                                                  entradas[2:],
                                                  reservatorio_esn,
                                                  modelo_preditor,
                                                  estimador_vazao)

    diferenca_predicao = predicao_matlab_k0 - predicao

    print(f'diferencas na predicao: \n {diferenca_predicao}')
    diferenca_maxima = max(diferenca_predicao)
    print(f'diferenca maxima na predicao: \n {diferenca_maxima}')

    # testar predicao com funcao casadi

    entrada_u_sym = ca.MX.sym('entrada_u', 1, len(entradas[:2]))
    entrada_x_sym = ca.MX.sym('entrada_x', 1, len(entradas[2:]))
    reservatorio_esn_sym = ca.MX.sym(
        'reservatorio_esn', reservatorio_esn.shape[0])

    predicao_sym, estado_reservatorio_sym = executa_predicao_casadi(entrada_u_sym,
                                                                    entrada_x_sym,
                                                                    reservatorio_esn_sym,
                                                                    modelo_preditor,
                                                                    estimador_vazao)

    preditor_BCSS = ca.Function('preditor_BCSS',
                                [entrada_u_sym, entrada_x_sym,
                                    reservatorio_esn_sym],
                                [predicao_sym, estado_reservatorio_sym],
                                ['entrada_u', 'entrada_x', 'reservatorio_esn'],
                                ['predicao', 'estado_reservatorio'])

    predicao_casadi, estado_reservatorio_casadi = preditor_BCSS(
        entradas[:2],
        entradas[2:],
        reservatorio_esn)
    
    diferenca_predicao = predicao_casadi.full().reshape(-1) - predicao

    print(f'diferencas na predicao casadi: \n {diferenca_predicao}')
    diferenca_maxima = max(diferenca_predicao)
    print(f'diferenca maxima na predicao casadi: \n {diferenca_maxima}')
