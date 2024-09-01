import pandas as pd
import numpy as np
from typing import Tuple
from icecream import ic
# Importando módulos necessários
import os
import sys
sys.path.append(os.path.abspath('codigos_python'))
import interpola
import avalia_condicao




def calcula_protecoes_mapas(
    tabela_simulador: pd.DataFrame,
    tabela_testes_producao: pd.DataFrame,
    frequencia: float,
    resolucao: float,
) -> Tuple[float, float, float, float, float, float]:
    """
    Calcula os limites dos mapas para uma determinada condição de frequência.

    Args:
    tabela_simulador (pd.DataFrame): Tabela do simulador
    tabela_testes_producao (pd.DataFrame): Tabela BTP
    frequencia (float): Frequência específica
    resolucao (float): Parâmetro de grid de pressão

    Returns:
    Tuple[float, float, float, float, float, float]:
        QMin, QMax, PSucMin, PSucMax, PChegadaMin, PChegadaMax
    """

    ic(tabela_simulador.info())
    for column in tabela_simulador.columns:
        ic(column)
        # Tenta converter a coluna para numérico
        try:
            tabela_simulador[column] = pd.to_numeric(tabela_simulador[column])
        except ValueError:
            # Se não for possível converter, mantém a coluna original
            print(f"Não foi possível converter a coluna '{column}' para numérico. Mantendo o tipo original.")
    ic(tabela_simulador.info())

    ic(tabela_testes_producao.info())
    for column in tabela_testes_producao.columns:
        ic(column)
        # Tenta converter a coluna para numérico
        try:
            tabela_testes_producao[column] = pd.to_numeric(tabela_testes_producao[column])
        except ValueError:
            # Se não for possível converter, mantém a coluna original
            print(f"Não foi possível converter a coluna '{column}' para numérico. Mantendo o tipo original.")
    ic(tabela_testes_producao.info())

    # Limites para as interpolações
    p_min = tabela_simulador["PressChegada"].min()
    p_max = tabela_simulador["PressChegada"].max()

    # Cria uma tabela temporária para avaliar as várias condições de
    # pressão de chegada
    t = pd.DataFrame(columns=tabela_simulador.columns)
    
    for p_chegada in np.arange(p_min, p_max + resolucao, resolucao):
        valores_interpolados = interpola.interpola(
            frequencia, p_chegada, tabela_simulador
        )
        print(valores_interpolados)
        t = pd.concat([t, valores_interpolados], ignore_index=True)

    # Avalia condição de Up e Downthrust
    resultado = avalia_condicao.avalia_condicao(t, tabela_testes_producao)
    condicao = resultado["Condicao"]

    print('apos condicao')

    # Define valores iniciais para os limites de Vazão, PSuc e PChegada
    q_min = float("inf")
    q_max = 0
    p_suc_min = float("inf")
    p_suc_max = 0
    p_chegada_min = float("inf")
    p_chegada_max = 0

    # Varre a tabela temporária para extrair os limites desejados
    for i in range(len(t)):
        if condicao[i] == "Normal":
            q_min = min(q_min, t["VazaoOleo"].iloc[i])
            q_max = max(q_max, t["VazaoOleo"].iloc[i])
            p_suc_min = min(p_suc_min, t["PressSuccao"].iloc[i])
            p_suc_max = max(p_suc_max, t["PressSuccao"].iloc[i])
            p_chegada_min = min(p_chegada_min, t["PressChegada"].iloc[i])
            p_chegada_max = max(p_chegada_max, t["PressChegada"].iloc[i])

    return q_min, q_max, p_suc_min, p_suc_max, p_chegada_min, p_chegada_max


# Exemplo de uso
if __name__ == "__main__":
    from carrega_tabelas import carrega_tabelas

    tabela_simulador, tabela_protecao_dinamica, tabela_faixas_percentuais, tabela_testes_producao = carrega_tabelas()

    frequencia = 55
    resolucao = 1

    q_min, q_max, p_suc_min, p_suc_max, p_chegada_min, p_chegada_max = (
        calcula_protecoes_mapas(
            tabela_simulador, tabela_testes_producao, frequencia, resolucao
        )
    )
    ic(q_min)
    ic(q_max)
    ic(p_suc_min)
    ic(p_suc_max)
    ic(p_chegada_min)
    ic(p_chegada_max)
