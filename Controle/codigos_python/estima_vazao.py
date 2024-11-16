import casadi as ca
import numpy as np
from typing import Tuple, Dict


def interpola_casadi_vazao(freq, press, matriz):
    """
    Realiza interpolação bilinear usando CasADi
    
    Parâmetros:
        Freq: Valor de frequência para interpolar (objeto simbólico CasADi)
        Press: Valor de pressão para interpolar (objeto simbólico CasADi)
        matriz: Matriz contendo os dados
    
    Retorna:
        float: Valor interpolado para VazaoOleo
    """
    # Busca pontos da vizinhança para proceder a interpolação bilinear
    pontos = sel_pontos_casadi_vazao(freq, press, matriz)
    
    # Obtém os pontos para interpolação
    f1 = pontos['frequencia_BCSS'][0]
    f2 = pontos['frequencia_BCSS'][2]
    p1 = pontos['pressao_chegada'][0]
    p2 = pontos['pressao_chegada'][1]
    
    # Realiza a interpolação bilinear
    return bilinear_casadi(
        freq, press,
        f1, f2, p1, p2,
        pontos['vazao_oleo'][0],
        pontos['vazao_oleo'][1],
        pontos['vazao_oleo'][2],
        pontos['vazao_oleo'][3]
    )

def bilinear_casadi(x, y, x1, x2, y1, y2, f11, f12, f21, f22):
    """
    Realiza interpolação bilinear usando CasADi
    
    Parâmetros:
        x, y: Pontos onde interpolar
        x1, x2: Valores x dos pontos conhecidos
        y1, y2: Valores y dos pontos conhecidos
        f11: Valor no ponto (x1,y1)
        f12: Valor no ponto (x1,y2)
        f21: Valor no ponto (x2,y1)
        f22: Valor no ponto (x2,y2)
    
    Retorna:
        float: Valor interpolado
    """
    # Interpolação BILINEAR
    k = (x2 - x1) * (y2 - y1)
    f = (f11*(x2-x)*(y2-y) + 
         f21*(x-x1)*(y2-y) + 
         f12*(x2-x)*(y-y1) + 
         f22*(x-x1)*(y-y1))
    
    f = ca.if_else(k != 0, f / k, f)
    
    # Casos especiais para interpolação LINEAR
    #cond_x = ca.logic_and(x1 == x2, y1 != y2)
    f_x = ca.if_else(ca.logic_and(x1 == x2, y1 != y2), f11 + (f12 - f11) * (y - y1) / (y2 - y1), f)
    
    #cond_y = ca.logic_and(y1 == y2, x1 != x2)
    f_y = ca.if_else(ca.logic_and(y1 == y2, x1 != x2), f11 + (f21 - f11) * (x - x1) / (x2 - x1), f)
    
    # Seleciona o resultado apropriado
    f = ca.if_else(x1 == x2, f_x, 
        ca.if_else(y1 == y2, f_y, f))
    
    # Para o caso em que os pontos são iguais aos conhecidos (não precisa interpolar)
    f = ca.if_else(ca.logic_and(x1 == x2, y1 == y2), f11, f)
    
    return f


from typing import Tuple, Dict
import casadi as ca
import numpy as np

def sel_pontos_casadi_vazao(freq: ca.SX, press: ca.SX, matriz: np.ndarray) -> Dict:
    """
    Seleciona 4 pontos da tabela para interpolação bilinear usando CasADi.
    
    Conhecido (freq, press), seleciona os 4 pontos da tabela para proceder a interpolação Bilinear.
    
    Args:
        freq: Valor de frequência para interpolar (objeto simbólico CasADi)
        press: Valor de pressão para interpolar (objeto simbólico CasADi)
        matriz: Array contendo os dados da tabela
    
    Returns:
        Dict contendo os 4 pontos selecionados para interpolação
    """
    # Extrai os vetores de Frequência e Pressão
    freq_bcss = matriz[:, 0]
    press_chegada = matriz[:, 1]
    
    # Obtém grids únicos para Frequência e Pressão
    grid_f = np.unique(freq_bcss)
    grid_p = np.unique(press_chegada)
    
    # Verifica e ajusta Frequência e Pressão se estiverem fora dos limites
    freq = verificar_limites(freq, grid_f)
    press = verificar_limites(press, grid_p)
    
    # Encontra pontos de interpolação para Frequência e Pressão
    f1, f2 = encontrar_pontos_interpolacao(freq, grid_f)
    p1, p2 = encontrar_pontos_interpolacao(press, grid_p)
    
    # Seleciona os 4 pontos para interpolação usando indexação simbólica
    vazao_oleo = ca.vertcat(
        selecionar_valor(matriz, p1, f1),
        selecionar_valor(matriz, p2, f1),
        selecionar_valor(matriz, p1, f2),
        selecionar_valor(matriz, p2, f2)
    )
    
    # Monta o dicionário de retorno com os pontos selecionados
    pontos = {
        'vazao_oleo': vazao_oleo,
        'frequencia_BCSS': ca.vertcat(f1, f1, f2, f2),
        'pressao_chegada': ca.vertcat(p1, p2, p1, p2)
    }
    
    return pontos

def verificar_limites(valor: ca.SX, grid: np.ndarray) -> ca.SX:
    """
    Verifica e ajusta um valor para estar dentro dos limites do grid.
    
    Args:
        valor: Valor a ser verificado
        grid: Array com os valores do grid
    
    Returns:
        Valor ajustado dentro dos limites
    """
    min_grid = grid[0]
    max_grid = grid[-1]
    
    return ca.if_else(valor < min_grid, min_grid,
                     ca.if_else(valor > max_grid, max_grid, valor))

def encontrar_pontos_interpolacao(valor: ca.SX, grid: np.ndarray) -> Tuple[ca.SX, ca.SX]:
    """
    Encontra os pontos inferior e superior para interpolação.
    Garante que os pontos retornados existem no grid original.
    
    Args:
        valor: Valor para encontrar os pontos de interpolação
        grid: Array com os valores do grid
    
    Returns:
        Tuple contendo os pontos inferior e superior para interpolação
    """
    # Inicializa com os valores extremos do grid
    inferior = grid[0]
    superior = grid[0]  # Inicializa com mesmo valor para caso de valor == grid[0]
    
    # Se o valor for igual a algum ponto do grid, retorna o mesmo ponto para ambos
    for i in range(len(grid)):
        cond_igual = (valor == grid[i])
        inferior = ca.if_else(cond_igual, grid[i], inferior)
        superior = ca.if_else(cond_igual, grid[i], superior)
    
    # Se o valor está entre pontos do grid, encontra os pontos adjacentes
    for i in range(len(grid)-1):
        cond_entre = ca.logic_and(valor > grid[i], valor < grid[i+1])
        inferior = ca.if_else(cond_entre, grid[i], inferior)
        superior = ca.if_else(cond_entre, grid[i+1], superior)
    
    # Caso o valor seja maior que o último ponto
    cond_maior = (valor > grid[-1])
    inferior = ca.if_else(cond_maior, grid[-1], inferior)
    superior = ca.if_else(cond_maior, grid[-1], superior)
    
    # Caso o valor seja menor que o primeiro ponto
    cond_menor = (valor < grid[0])
    inferior = ca.if_else(cond_menor, grid[0], inferior)
    superior = ca.if_else(cond_menor, grid[0], superior)
    
    return inferior, superior

def selecionar_valor(matriz: np.ndarray, press: ca.SX, freq: ca.SX) -> ca.SX:
    """
    Seleciona um valor da matriz baseado na pressão e frequência.
    
    Args:
        matriz: Array contendo os dados
        press: Valor de pressão
        freq: Valor de frequência
    
    Returns:
        Valor selecionado da matriz
    """
    #valor = 0
    valor = matriz[0,2]
    for i in range(matriz.shape[0]):
        cond = ca.logic_and(matriz[i,0] == freq, matriz[i,1] == press)
        valor = ca.if_else(cond, matriz[i,2], valor)
    
    return valor

def funcao_interpolacao_casadi(matriz):
    """
    Fixture que cria a função de interpolação CasADi usando dados do Excel
    """
    freq_sym = ca.MX.sym('freq')
    press_sym = ca.MX.sym('press')
    
    return ca.Function('interpolacao',
                         [freq_sym, press_sym],
                         [interpola_casadi_vazao(freq_sym, press_sym, matriz)],
                         ['freq', 'press'],
                         ['vazao'])