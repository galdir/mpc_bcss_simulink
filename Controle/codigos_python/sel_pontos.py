import pandas as pd
import numpy as np


def sel_pontos(Freq, Press, T):
    """
    Conhecido (Freq,Press), ponto no qual se quer calcular os dados,
    Seleciona os 4 pontos da tabela T para proceder a interpolação Bilinear
    """
    # Inicializa Pontos com a primeira linha de T
    Pontos = T.iloc[[0]].copy()

    # Confere os pontos que fazem parte do grid de simulação da Frequencia
    gridF = sorted(T["FreqBCSS"].unique())

    # Verifica se o valor de frequencia está dentro do grid para não dar erro de interpolação
    if Freq < min(gridF) or Freq > max(gridF):
        print(
            f"Ponto de Frequencia {Freq:.1f}Hz está fora da faixa de interpolação [{min(gridF):.0f} {max(gridF):.0f}] Hz"
        )
        if Freq < min(gridF):
            Freq = min(gridF)
        if Freq > max(gridF):
            Freq = max(gridF)
        print(f"Cálculo feito com base na Frequencia = {Freq:.0f}Hz")
        # Aqui você pode adicionar um som de beep se necessário

    if Freq in gridF:
        F = [Freq, Freq]
    else:
        pos = next(i for i, v in enumerate(gridF) if v > Freq)
        F = [gridF[pos - 1], gridF[pos]]

    # Confere os pontos que fazem parte do grid de simulação da Pressão de Chegada
    gridP = sorted(T["PressChegada"].unique())

    if Press < min(gridP) or Press > max(gridP):
        print(
            f"Ponto P.Chegada {Press:.1f} Kgf/cm2 está fora da faixa de interpolação [{min(gridP):.0f} {max(gridP):.0f}] Kgf/cm2"
        )
        if Press < min(gridP):
            Press = min(gridP)
        if Press > max(gridP):
            Press = max(gridP)
        print(f"Cálculo feito com base na Pressão de Chegada = {Press:.0f}Kgf/cm2")

    if Press in gridP:
        P = [Press, Press]
    else:
        pos = next(i for i, v in enumerate(gridP) if v > Press)
        P = [gridP[pos - 1], gridP[pos]]

    # Seleciona os pontos para interpolação bilinear
    Pontos = pd.concat(
        [
            T[(T["FreqBCSS"] == F[0]) & (T["PressChegada"] == P[0])],
            T[(T["FreqBCSS"] == F[0]) & (T["PressChegada"] == P[1])],
            T[(T["FreqBCSS"] == F[1]) & (T["PressChegada"] == P[0])],
            T[(T["FreqBCSS"] == F[1]) & (T["PressChegada"] == P[1])],
        ]
    ).reset_index(drop=True)

    return Pontos


# Exemplo de uso
if __name__ == "__main__":
    TabSimulador = pd.read_excel(
        "../Dados/DoSimulador.xlsx"
    )  # Carregue seus dados aqui
    TabSimulador = TabSimulador.iloc[1:]
    TabSimulador.columns = [
        "FreqBCSS",
        "PressChegada",
        "VazaoOleo",
        "VazaoLiquido",
        "Twh",
        "Pwh",
        "DeltaP",
    ]
    for coluna in TabSimulador.columns:
        TabSimulador[coluna] = TabSimulador[coluna].astype("float")
    # Cria coluna de Pressão de Sucção com base na Pressão de Descarga e no DeltaP
    TabSimulador["PressSuccao"] = TabSimulador.Pwh - TabSimulador.DeltaP
    Freq = 50
    Press = 35
    Pontos = sel_pontos(Freq, Press, TabSimulador)
    print(Pontos)
