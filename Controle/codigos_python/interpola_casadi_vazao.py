import casadi as cs
import numpy as np
import pandas as pd

def interpola_casadi_vazao(Freq, Press, matriz):
    """
    Interpola_casadi_vazao Realiza interpolação bilinear usando CasADi

    Interpola valores baseados em Frequência e Pressão usando CasADi

    Entradas:
        Freq   - Valor de frequência para interpolar (objeto simbólico CasADi)
        Press  - Valor de pressão para interpolar (objeto simbólico CasADi)
        matriz - matriz contendo os dados de freq, press e vazao

    Saída:
        New - Valor interpolado de VazaoOleo
    """
    # Busca pontos da vizinhança para proceder a interpolação bilinear
    Pontos = selPontos_casadi_vazao(Freq, Press, matriz)

    # Realiza a interpolação bilinear para VazaoOleo
    campo = 'VazaoOleo'
    f1 = Pontos['FreqBCSS'][0]
    f2 = Pontos['FreqBCSS'][2]
    p1 = Pontos['PressChegada'][0]
    p2 = Pontos['PressChegada'][1]
    New = bilinear_casadi(Freq, Press, f1, f2, p1, p2, 
                          Pontos[campo][0], Pontos[campo][1], 
                          Pontos[campo][2], Pontos[campo][3])

    return New

def bilinear_casadi(x, y, x1, x2, y1, y2, f11, f12, f21, f22):
    """
    Bilinear_casadi Realiza interpolação bilinear usando CasADi
    """
    # Interpolação BILINEAR
    K = (x2 - x1) * (y2 - y1)
    f = f11*(x2-x)*(y2-y) + f21*(x-x1)*(y2-y) + f12*(x2-x)*(y-y1) + f22*(x-x1)*(y-y1)
    f = f / K
    
    # Casos especiais para interpolação LINEAR
    f_x = cs.if_else(x1 == x2, f11 + (f12 - f11) * (y - y1) / (y2 - y1), f)
    
    f_y = cs.if_else(y1 == y2, f11 + (f21 - f11) * (x - x1) / (x2 - x1), f)
    
    # Seleciona o resultado apropriado
    f = cs.if_else(x1 == x2, f_x, cs.if_else(y1 == y2, f_y, f))
    f = cs.if_else(x1 == x2, cs.if_else(y1 == y2, f11, f), f)
    
    return f

# Nota: A função selPontos_casadi_vazao precisa ser implementada separadamente
# ou importada de outro módulo onde já tenha sido definida.



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
        cond = cs.logic_and(valor >= grid[i], valor < grid[i+1])
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
            cond = cs.logic_and(gridP[i] == P, gridF[j] == F)
            valor = cs.if_else(cond, matriz[i,j], valor)
    return valor


# Exemplo de uso
if __name__ == "__main__":
    df_simulador = pd.read_excel('./Tabelas/DoSimulador.xlsx')  
    
    #matriz_LimitesDinamicos_vazao = df_LimitesDinamicos.drop('LIMITES', axis=1).values
    matriz_simulador_vazao = df_simulador.iloc[1:,:3].values

    # matriz_teste = np.array([
    #     [30, 10, 100],
    #     [30, 20, 150],
    #     [40, 10, 200],
    #     [40, 20, 250],
    #     [50, 10, 300],
    #     [50, 20, 350],
    # ])
   
    # Criar símbolos CasADi para Freq e Press
    Freq_sym = cs.MX.sym('Freq')
    Press_sym = cs.MX.sym('Press')

    # Criar uma função CasADi
    interpola_func = cs.Function('interpola', [Freq_sym, Press_sym], 
                                [interpola_casadi_vazao(Freq_sym, Press_sym, matriz_simulador_vazao)])

    freq = 50
    press = 35

    # Avaliar a função
    resultado = interpola_func(freq, press)
    print(f"Para Freq = {freq} e Press = {press}")
    
    print(f"VazaoOleo interpolada: {resultado}")    