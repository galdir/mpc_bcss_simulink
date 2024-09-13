import json
import pandas as pd

# Importando módulos necessários
import os
import sys

sys.path.append(os.path.abspath("codigos_python"))
from protecoes_mapas import calcula_protecoes_mapas
from protecao_dinamica import calcula_protecao_dinamica


def calcula_limites(
    frequencia: float,
    tabela_simulador: pd.DataFrame,
    tabela_testes_producao: pd.DataFrame,
    tabela_dp: pd.DataFrame,
    tabela_faixas: pd.DataFrame,
    tabela_protecao_fixa: pd.DataFrame,
):
    """
    Esta função retorna uma tabela com os valores limites para as variáveis do processo.
    Os limites podem ser extraídos das tabelas de proteção dinâmica, podem ter valores fixos (pré-defindos),
    como também podem vir dos mapas de operação.

    =================================
    PROTECAO FIXA - Proteções pré-definidas
    De uma tabela definida pela Petrobras, usamos os limites fixos para as seguintes variáveis:
    1 - Limites H e L da Temperatura do Motor
    2 - Limites H e L da Temperatura de Sucção
    3 - Limites H e L da Vibração
    ProtecaoFixa =readtable('FixedProtections.xlsx');

    ===============================
    PROTECAO 2 - Proteções Dinâmicas
    Da tabela de proteção dinâmica da Petrobras, usamos com limites das variáveis:
    1 - Limites de alarme H e L da Corrente Total da BCSS
    2 - Limites de alarme H e L da Corrente de Torque da BCSS
    3 - Limites de alarme H e L  da Pressão de Sucção de BCSS
    4 - Limites de alarme H e L  da Pressão de Descarga da BCSS
    5 - Limites de alarme H e L  da Pressão Diferencial da BCSS
    6 - Limites de alarme H e L da Pressão de Chegada
    7 - Limites de alarme H e L da Temperatura de Chegada

    """

    df_protecao_dinamica = calcula_protecao_dinamica(
        frequencia, tabela_dp, tabela_faixas
    )  # define as restrições HARD dos estados em função da frequencia = obj.uk(1)'

    df_protecao_dinamica.iloc[1, 0] = 0  # Define o primeiro valor como 0
    df_protecao_dinamica.iloc[1, 1:] = df_protecao_dinamica.iloc[0, 1:] * (
        1 - tabela_faixas.iloc[1, 1:]
    )

    df_protecao_dinamica.iloc[2, 0] = 0  # Define o primeiro valor como 0
    df_protecao_dinamica.iloc[2, 1:] = df_protecao_dinamica.iloc[0, 1:] * (
        1 + tabela_faixas.iloc[2, 1:]
    )

    df_limites = df_protecao_dinamica.iloc[
        1:3, 1:
    ].reset_index()  # Gera tabela unificadas com todos os limites calculados
    df_limites_inv = df_limites.copy()
    df_limites_inv.iloc[0] = df_limites.iloc[1]
    df_limites_inv.iloc[1] = df_limites.iloc[0]

    #tabela_protecao_fixa = pd.concat([tabela_protecao_fixa.iloc[:, 1:], df_limites_inv], axis=1)
    df_limites_inv = pd.concat([tabela_protecao_fixa, df_limites_inv.iloc[:, 1:]], axis=1)

    # ============================
    # PROTECAO 3 - Mapa de operação
    resolucao_grid = 1
    # Grid de pressão para maior precisão na interpolação dos limites dos mapas de operação

    resultado = calcula_protecoes_mapas(
        tabela_simulador, tabela_testes_producao, frequencia, resolucao_grid
    )
    # % define as restrições suaves (estratégia por faixa do MPC) em função da frequencia
    vazao_min = resultado[0]
    vazao_max = resultado[1]
    pressao_succao_min = resultado[2]
    pressao_succao_max = resultado[3]
    pressao_chegada_min = resultado[4]
    pressao_chegada_max = resultado[5]

    # Necessário lembrar que no MAPA as contas de pressão são feitas em kgf/cm2
    # ou seja, precisam ser convertidas para bar (1bar = 1.019716 kgf/cm2)
    pressao_succao_min = pressao_succao_min / 1.019716
    pressao_succao_max = pressao_succao_max / 1.019716
    pressao_chegada_min = pressao_chegada_min / 1.019716
    pressao_chegada_max = pressao_chegada_max / 1.019716

    # =====================================================
    # Substitui considerando os limites do mapa
    # df_limites_inv.ProductionSurfacePressure.iloc[0] = min(df_limites_inv.ProductionSurfacePressure.iloc[0], pressao_chegada_max)
    # df_limites_inv.ProductionSurfacePressure.iloc[1] = max(df_limites_inv.ProductionSurfacePressure.iloc[1], pressao_chegada_min)

    df_limites_inv.loc[df_limites_inv.index[0], "ProductionSurfacePressure"] = min(
        df_limites_inv.ProductionSurfacePressure.iloc[0], pressao_chegada_max
    )
    df_limites_inv.loc[df_limites_inv.index[1], "ProductionSurfacePressure"] = max(
        df_limites_inv.ProductionSurfacePressure.iloc[1], pressao_chegada_min
    )

    # df_limites_inv.IntakePressure.iloc[0] = min(df_limites_inv.IntakePressure.iloc[0], pressao_succao_max)
    # df_limites_inv.IntakePressure.iloc[1] = max(df_limites_inv.IntakePressure.iloc[1], pressao_succao_min)

    df_limites_inv.loc[df_limites_inv.index[0], "IntakePressure"] = min(
        df_limites_inv.IntakePressure.iloc[0], pressao_succao_max
    )
    df_limites_inv.loc[df_limites_inv.index[1], "IntakePressure"] = max(
        df_limites_inv.IntakePressure.iloc[1], pressao_succao_min
    )

    #df_limites_inv.rename(columns={"index": "LIMITES"}, inplace=True)
    return df_limites_inv


def calcula_limites_json(
    frequencia,
    tabela_simulador_json,
    tabela_testes_producao_json,
    tabela_dp_json,
    tabela_faixas_json,
    tabela_protecao_fixa_json,
):
    # print(texto_json)
    tabela_simulador_dict = json.loads(tabela_simulador_json)
    # print(tabela_simulador_dict[0])
    tabela_simulador_df = pd.DataFrame(tabela_simulador_dict)

    tabela_testes_producao_dict = json.loads(tabela_testes_producao_json)
    tabela_testes_producao_df = pd.DataFrame([tabela_testes_producao_dict])

    tabela_dp_dict = json.loads(tabela_dp_json)
    tabela_dp_df = pd.DataFrame(tabela_dp_dict)

    tabela_faixas_dict = json.loads(tabela_faixas_json)
    tabela_faixas_df = pd.DataFrame(tabela_faixas_dict)

    tabela_protecao_fixa_dict = json.loads(tabela_protecao_fixa_json)
    tabela_protecao_fixa_df = pd.DataFrame(tabela_protecao_fixa_dict)

    # print(df)
    return calcula_limites(
        frequencia = frequencia,
        tabela_simulador=tabela_simulador_df,
        tabela_testes_producao=tabela_testes_producao_df,
        tabela_dp = tabela_dp_df,
        tabela_faixas = tabela_faixas_df,
        tabela_protecao_fixa = tabela_protecao_fixa_df
    )


# Exemplo de uso
if __name__ == "__main__":
    from carrega_tabelas import carrega_tabelas

    (
        tabela_simulador,
        tabela_protecao_dinamica,
        tabela_faixas_percentuais,
        tabela_testes_producao,
        tabela_protecao_fixa,
    ) = carrega_tabelas(
        # caminho_tabela_simulador="./Dados/DoSimulador.xlsx",
        caminho_tabela_simulador="./Dados/DoSimulador - ate_pchegada_51.xlsx",
        caminho_tabela_testes_producao="./Dados/DoBTP.xlsx",
        caminho_tabela_proteca_dinamica="./Dados/DP.xlsx",
        caminho_tabela_faixas_percentuais="./Dados/DP-Faixas.xlsx",
        caminho_tabela_protecao_fixa="./Dados/FixedProtections.xlsx",
    )

    frequencia = 40

    df_limites = calcula_limites(
        frequencia,
        tabela_simulador,
        tabela_testes_producao,
        tabela_protecao_dinamica,
        tabela_faixas_percentuais,
        tabela_protecao_fixa,
    )

    print(df_limites)
