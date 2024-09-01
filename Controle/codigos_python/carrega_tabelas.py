import pandas as pd


def carrega_tabelas(
    caminho_tabela_simulador="../Dados/DoSimulador.xlsx",
    caminho_tabela_testes_producao="../Dados/DoBTP.xlsx",
    caminho_tabela_proteca_dinamica="../Dados/DP.xlsx",
    caminho_tabela_faixas_percentuais="../Dados/DP-Faixas.xlsx",
):
    """
    Carrega as tabelas de referência

    Args:
        caminho_tabela_simulador(str): Caminho par ao arquivo excel com os dados de testes de simulador
        caminho_tabela_proteca_dinamica (str): Caminho para o arquivo Excel com os dados de proteção dinâmica.
        caminho_tabela_faixas_percentuais (str): Caminho para o arquivo Excel com as faixas percentuais.

    Returns:
        tuple: Tabela de proteção dinâmica e tabela de faixas percentuais.
    """


    tabela_simulador = pd.read_excel(
        caminho_tabela_simulador
    )  # Carregue seus dados aqui
    tabela_simulador = tabela_simulador.iloc[1:]
    tabela_simulador.columns = [
        "FreqBCSS",
        "PressChegada",
        "VazaoOleo",
        "VazaoLiquido",
        "Twh",
        "Pwh",
        "DeltaP",
    ]
    for coluna in tabela_simulador.columns:
        tabela_simulador[coluna] = tabela_simulador[coluna].astype("float")
    # Cria coluna de Pressão de Sucção com base na Pressão de Descarga
    # e no DeltaP
    tabela_simulador["PressSuccao"] = tabela_simulador.Pwh - tabela_simulador["DeltaP"]

    tabela_protecao_dinamica = pd.read_excel(caminho_tabela_proteca_dinamica)
    tabela_protecao_dinamica = tabela_protecao_dinamica.loc[2:].reset_index(drop=True)
    for coluna in tabela_protecao_dinamica.columns:
        tabela_protecao_dinamica[coluna] = tabela_protecao_dinamica[coluna].astype(
            "float"
        )

    tabela_faixas_percentuais = pd.read_excel(caminho_tabela_faixas_percentuais)
    tabela_faixas_percentuais = tabela_faixas_percentuais.loc[1:].reset_index(drop=True)

    for coluna in tabela_faixas_percentuais.columns[1:]:
        tabela_faixas_percentuais[coluna] = tabela_faixas_percentuais[coluna].astype(
            "float"
        )

    tabela_testes_producao = pd.read_excel(caminho_tabela_testes_producao)
    tabela_testes_producao = tabela_testes_producao.iloc[1:].reset_index(drop=True)

    return tabela_simulador, tabela_protecao_dinamica, tabela_faixas_percentuais, tabela_testes_producao


