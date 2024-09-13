import pandas as pd
import numpy as np
import json


def carregar_tabelas_local(
    caminho_dp="../Dados/DP.xlsx", caminho_faixas="../Dados/DP-Faixas.xlsx"
):
    """
    Carrega as tabelas de referência para as proteções dinâmicas.

    Args:
        caminho_dp (str): Caminho para o arquivo Excel com os dados de proteção dinâmica.
        caminho_faixas (str): Caminho para o arquivo Excel com as faixas percentuais.

    Returns:
        tuple: Tabela de proteção dinâmica e tabela de faixas percentuais.
    """
    tabela_dp = pd.read_excel(caminho_dp)
    tabela_dp = tabela_dp.loc[2:].reset_index(drop=True)
    for coluna in tabela_dp.columns:
        tabela_dp[coluna] = tabela_dp[coluna].astype("float")
    tabela_faixas = pd.read_excel(caminho_faixas)
    tabela_faixas = tabela_faixas.loc[1:].reset_index(drop=True)
    for coluna in tabela_faixas.columns[1:]:
        tabela_faixas[coluna] = tabela_faixas[coluna].astype("float")

    return tabela_dp, tabela_faixas


def interpolar_valores(frequencia, linha_inferior, linha_superior):
    """
    Realiza interpolação linear entre duas linhas da tabela.

    Args:
        frequencia (float): Frequência para interpolação.
        linha_inferior (pd.Series): Linha com valores inferiores.
        linha_superior (pd.Series): Linha com valores superiores.

    Returns:
        dict: Dicionário com valores interpolados.
    """
    resultado = {}
    for coluna in linha_inferior.index:
        if coluna == "FreqBCSS":
            resultado[coluna] = frequencia
        else:
            x1, x2 = linha_inferior["FreqBCSS"], linha_superior["FreqBCSS"]
            y1, y2 = linha_inferior[coluna], linha_superior[coluna]
            resultado[coluna] = calcular_interpolacao_linear(frequencia, x1, x2, y1, y2)
    return resultado


def calcular_interpolacao_linear(x, x1, x2, y1, y2):
    """
    Calcula a interpolação linear para um ponto.

    Args:
        x (float): Valor x para interpolação.
        x1, x2 (float): Valores x conhecidos.
        y1, y2 (float): Valores y conhecidos.

    Returns:
        float: Valor y interpolado.
    """
    coeficiente_angular = (y2 - y1) / (x2 - x1)
    coeficiente_linear = y1 - coeficiente_angular * x1
    return coeficiente_angular * x + coeficiente_linear


def calcula_protecao_dinamica(frequencia=None, tabela_dp=None, tabela_faixas=None):
    """
    Calcula os limites de alarmes para variáveis baseadas na frequência de operação.

    Args:
        frequencia (float): Frequência de operação. Se None, usa 40.4 Hz.
        tabela_dp (pd.DataFrame): Tabela de proteção dinâmica. Se None, carrega do arquivo.
        tabela_faixas (pd.DataFrame): Tabela de faixas percentuais. Se None, carrega do arquivo.

    Returns:
        pd.DataFrame: DataFrame com os valores calculados e limites de alarme.
    """
    # Carrega as tabelas se não fornecidas
    if tabela_dp is None or tabela_faixas is None:
        tabela_dp, tabela_faixas = carregar_tabelas_local()

    # Define frequência padrão se não fornecida
    if frequencia is None:
        frequencia = 40.4

    # Garante que a frequência esteja dentro do intervalo da tabela
    frequencia = np.clip(
        frequencia, tabela_dp["FreqBCSS"].min(), tabela_dp["FreqBCSS"].max()
    )

    # Encontra a posição mais próxima na tabela
    posicao = np.abs(tabela_dp["FreqBCSS"] - frequencia).idxmin()

    # Determina se é necessário interpolar
    if tabela_dp["FreqBCSS"].iloc[posicao] == frequencia:
        resultado = tabela_dp.iloc[posicao].to_dict()
    elif posicao == len(tabela_dp) - 1:
        resultado = tabela_dp.iloc[-1].to_dict()
    else:
        resultado = interpolar_valores(
            frequencia, tabela_dp.iloc[posicao], tabela_dp.iloc[posicao + 1]
        )

    # Calcula os limites inferior (L) e superior (H)
    colunas_comuns = list(
        (set(resultado.keys()) & set(tabela_faixas.columns)) - set(["FreqBCSS"])
    )
    limite_inferior = {
        col: resultado[col] * (1 - tabela_faixas.iloc[1][col]) for col in colunas_comuns
    }
    limite_superior = {
        col: resultado[col] * (1 + tabela_faixas.iloc[2][col]) for col in colunas_comuns
    }
    limite_inferior["FreqBCSS"] = 0
    limite_superior["FreqBCSS"] = 0
    return pd.DataFrame(
        [resultado, limite_inferior, limite_superior],
        index=["Valor", "Limite Inferior", "Limite Superior"],
    )


def calcula_protecao_dinamica_json(frequencia, tabela_dp_json, tabela_faixas_json):
    #print(texto_json)
    tabela_dp_dict = json.loads(tabela_dp_json)
    tabela_dp_df = pd.DataFrame(tabela_dp_dict)

    tabela_faixas_dict = json.loads(tabela_faixas_json)
    tabela_faixas_df = pd.DataFrame(tabela_faixas_dict)

    #print(df)
    return calcula_protecao_dinamica(frequencia=frequencia, tabela_dp=tabela_dp_df, tabela_faixas=tabela_faixas_df)


# Exemplo de uso
if __name__ == "__main__":
    frequencia_teste = 40
    resultado = calcula_protecao_dinamica(frequencia_teste)
    print(f"Resultados para frequência de {frequencia_teste} Hz:")
    print(resultado)
