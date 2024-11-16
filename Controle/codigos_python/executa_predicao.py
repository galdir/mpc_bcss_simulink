import numpy as np

from estima_vazao import funcao_interpolacao_casadi
from normalizacao import desnormaliza_predicoes, normaliza_entradas


def executa_predicao(EntradaU, EstadosX, ESNdataa0, modelo_ESN, f_matrizVazao_sym):
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
    predicoes, novo_a0 = executa_predicao_esn(EntradaU, EstadosX, ESNdataa0, modelo_ESN)
    
    # Executa predição da vazão
    Freq = EntradaU[0]
    PChegada = predicoes[1] * conversao_bar_kgf  # Converte pressão de bar para Kgf/cm2
    vazaoOleo_estimada = f_matrizVazao_sym(Freq, PChegada)
    
    # Monta vetor final incluindo a vazão estimada
    predicoes = np.append(predicoes, vazaoOleo_estimada)
    
    return predicoes, novo_a0

def executa_predicao_esn(EntradaU, EstadosX, ESNdataa0, modelo_ESN):
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
    # Monta vetor de entrada (excluindo vazão)
    entradas = np.concatenate([EntradaU, EstadosX[:-1]])
    
    # Normaliza entradas
    entradas_normalizadas = normaliza_entradas(entradas)
    
    # Calcula novo estado do reservatório
    x_ESN = (modelo_ESN['data']['Wrr'][0][0] @ ESNdataa0 + 
             modelo_ESN['data']['Wir'][0][0] @ entradas_normalizadas + 
             modelo_ESN['data']['Wbr'][0][0])
    
    # Atualiza estado da ESN
    novo_a0 = ((1 - modelo_ESN['data']['gama'][0][0]) * ESNdataa0 + 
               modelo_ESN['data']['gama'][0][0] * np.tanh(x_ESN))
    
    # Adiciona bias
    a_wbias = np.concatenate([[1.0], novo_a0])
    
    # Calcula predições
    predicoes_normalizadas = modelo_ESN['data']['Wro'][0][0] @ a_wbias
    
    # Desnormaliza as predições
    predicoes = desnormaliza_predicoes(predicoes_normalizadas)
    
    return predicoes, novo_a0


# Exemplo de uso
if __name__ == "__main__":
    import scipy.io as sio
    from pathlib import Path
    import pandas as pd

    # Nome do arquivo .mat a ser carregado
    nome_esn = 'weightsESNx_TR400_TVaz0.9_RaioE0.4.mat'
    caminho_modelos = Path("./Modelos/ESN/")
    caminho_modelo_esn = caminho_modelos / nome_esn

    arquivo_matriz_vazao = 'DoSimulador.xlsx'
    caminho_tabelas = Path("./Tabelas")
    df_matriz_vazao = pd.read_excel(caminho_tabelas / arquivo_matriz_vazao, skiprows=2, header=None)
    matriz = df_matriz_vazao.iloc[:,:3].values
    # Carrega o arquivo .mat
    modelo_preditor = sio.loadmat(caminho_modelo_esn)

    estimador_vazao = funcao_interpolacao_casadi(matriz)

    pasta_dados = Path('C:\\petrobras_2023_sistema_controle_inteligente_operacao_BCS_campo\\DADOS UTEIS\\dados MPA OPC\\')
    nome_arquivo_dados = 'df_opc_mpa_11_12s.parquet'
    #nome_arquivo_dados = 'df_opc_mpa_10s.parquet'

    df = pd.read_parquet(pasta_dados / nome_arquivo_dados)
    periodo = { 'inicio': '2024-03-10 18:00:00',
                'fim' : '2024-03-11 08:00:00'}
    
    df_periodo = df.query('index > @periodo["inicio"] and index < @periodo["fim"]').copy()
    
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
    
    #estima vazao
    vazao = float(estimador_vazao(df_periodo['frequencia_BCSS'].iloc[0], df_periodo['pressao_montante_alvo'].iloc[0]))
    df_periodo['vazao_oleo'] = vazao

    variaveis_preditoras = variaveis_manipuladas + variaveis_processo

    #df_periodo_manipuladas = df_periodo[variaveis_manipuladas]
    #df_periodo_processo = df_periodo[variaveis_processo]

    df_periodo_preditoras = df_periodo[variaveis_preditoras]

    nsim = len(df_periodo)
    predicao = np.zeros([nsim, len(variaveis_preditoras)])

    # =========================================================================
    # Inicia a simulação da planta BCS para gerar os dados  
    # =========================================================================
    repositorio_esn = modelo_preditor['data']['a0'][0][0]
    for k in range(nsim):
        entradas = df_periodo_preditoras.iloc[k].values
        #entradas_normalizadas = normaliza_entradas(df_periodo_preditoras.iloc[k].values)
         
        predicao_normalizada, repositorio_esn = executa_predicao(entradas[:2], 
                                                                 entradas[2:], 
                                                                 repositorio_esn, 
                                                                 modelo_preditor,
                                                                 estimador_vazao)
        
        predicao[k] = desnormaliza_predicoes(predicao_normalizada)
