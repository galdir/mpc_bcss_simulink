import casadi as cs
import numpy as np



def selPontos_casadi_vazao(Freq, Press, matriz):
    """
    selPontos_casadi_vazao Seleciona 4 pontos da tabela T para interpolação bilinear usando CasADi

    Conhecido (Freq,Press), seleciona os 4 pontos da tabela T para proceder a interpolação Bilinear

    Entradas:
        Freq     - Valor de frequência para interpolar (objeto simbólico CasADi)
        Press    - Valor de pressão para interpolar (objeto simbólico CasADi)
        matriz   - matriz contendo os dados de freq, press e vazao

    Saída:
        Pontos - Dicionário contendo os 4 pontos selecionados para interpolação
    """

    # Extrai os vetores de Frequência e Pressão
    FreqBCSS = matriz[:, 0]
    PressChegada = matriz[:, 1]

    # Obtém grids únicos para Frequência e Pressão
    gridF = np.unique(FreqBCSS)
    gridP = np.unique(PressChegada)

    # Verifica e ajusta Frequência e Pressão se estiverem fora dos limites
    Freq = verificarLimites(Freq, gridF)
    Press = verificarLimites(Press, gridP)

    # Encontra pontos de interpolação para Frequência e Pressão
    F1, F2 = encontrarPontosInterpolacao(Freq, gridF)
    P1, P2 = encontrarPontosInterpolacao(Press, gridP)

    # Inicializa dicionário para armazenar os pontos
    Pontos = {}

    valores = matriz[:, 2]

    # Reorganiza os valores em uma matriz 2D
    valores_2d = valores.reshape(len(gridP), len(gridF))

    # Seleciona os 4 pontos para interpolação usando indexação simbólica
    Pontos['VazaoOleo'] = cs.vertcat(
        selecionarValor(valores_2d, gridP, gridF, P1, F1),
        selecionarValor(valores_2d, gridP, gridF, P2, F1),
        selecionarValor(valores_2d, gridP, gridF, P1, F2),
        selecionarValor(valores_2d, gridP, gridF, P2, F2)
    )

    # Adiciona Freq e Press selecionados
    Pontos['FreqBCSS'] = cs.vertcat(F1, F1, F2, F2)
    Pontos['PressChegada'] = cs.vertcat(P1, P2, P1, P2)

    return Pontos

def verificarLimites(valor, grid):
    minGrid = grid[0]
    maxGrid = grid[-1]
    
    return cs.if_else(valor < minGrid, minGrid,
             cs.if_else(valor > maxGrid, maxGrid, valor))

def encontrarPontosInterpolacao(valor, grid):
    n = len(grid)
    inferior = grid[0]
    superior = grid[-1]
    
    for i in range(n-1):
        cond = (valor >= grid[i]) & (valor < grid[i+1])
        inferior = cs.if_else(cond, grid[i], inferior)
        superior = cs.if_else(cond, grid[i+1], superior)

        # caso o valor buscado seja idêntico a um já existente
        superior = cs.if_else(valor == grid[i], grid[i], superior)
    
    # Tratamento especial para o último ponto do grid
    cond_last = (valor == grid[-1])
    inferior = cs.if_else(cond_last, grid[-2], inferior)
    superior = cs.if_else(cond_last, grid[-1], superior)

    return inferior, superior

def selecionarValor(matriz, gridP, gridF, P, F):
    valor = 0
    for i in range(len(gridP)):
        for j in range(len(gridF)):
            cond = (gridP[i] == P) & (gridF[j] == F)
            valor = cs.if_else(cond, matriz[i,j], valor)
    return valor