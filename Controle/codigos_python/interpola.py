import pandas as pd
import numpy as np
from sel_pontos import sel_pontos

def interpola(freq, press, T, indice=None):
    """
    Busca pontos da vizinhança para proceder a interpolação bilinear.
    
    :param freq: Frequência para interpolação
    :param press: Pressão para interpolação
    :param T: DataFrame com os dados
    :param indice: Índice específico para retornar (opcional)
    :return: DataFrame ou valor específico interpolado
    """
    # Assumindo que a função selPontos está definida em outro lugar
    pontos = sel_pontos(freq, press, T)
    
    if len(pontos) == 4:  # Os 4 pontos foram selecionados
        f1 = pontos['FreqBCSS'].iloc[0]
        f2 = pontos['FreqBCSS'].iloc[2]
        p1 = pontos['PressChegada'].iloc[0]
        p2 = pontos['PressChegada'].iloc[1]
        
        # Executa a interpolação bilinear para cada uma das variáveis
        variaveis = ['VazaoOleo', 'VazaoLiquido', 'Twh', 'Pwh', 'DeltaP', 'PressSuccao']
        new_data = {}
        for var in variaveis:
            new_data[var] = bilinear(freq, press, f1, f2, p1, p2, 
                                     pontos[var].iloc[0], pontos[var].iloc[1], 
                                     pontos[var].iloc[2], pontos[var].iloc[3])
        
        new = pontos.iloc[[0]].copy()  # Cria uma cópia da primeira linha para manter a estrutura
        
        # Atualiza dados do registro com os valores interpolados
        new['FreqBCSS'] = freq
        new['PressChegada'] = press
        for var, value in new_data.items():
            new[var] = value
        
        if indice is not None:
            return new.iloc[0, indice]
        else:
            return new
    else:
        return None  # Retorna None se não encontrou 4 pontos

def bilinear(x, y, x1, x2, y1, y2, f11, f12, f21, f22):
    """
    Realiza interpolação bilinear.
    
    :param x, y: Pontos para interpolação
    :param x1, x2, y1, y2: Coordenadas dos pontos conhecidos
    :param f11, f12, f21, f22: Valores nos pontos conhecidos
    :return: Valor interpolado
    """
    # Se os pontos de Frequência e Pressão já existem no grid da simulação
    if x1 == x2 and y1 == y2:
        return f11  # Qualquer um dos 4 pontos teriam o mesmo valor
    
    # Interpolação BILINEAR
    if x1 != x2 and y1 != y2:
        K = (x2 - x1) * (y2 - y1)
        f = (f11*(x2-x)*(y2-y) + f21*(x-x1)*(y2-y) + 
             f12*(x2-x)*(y-y1) + f22*(x-x1)*(y-y1)) / K
    
    # Interpolação LINEAR com base na Pressão
    elif x1 == x2 and y1 != y2:
        a = (f12 - f11) / (y2 - y1)
        b = f11 - a * y1
        f = a * y + b
    
    # Interpolação LINEAR com base na Frequência
    elif x1 != x2 and y1 == y2:
        a = (f22 - f11) / (x2 - x1)
        b = f11 - a * x1
        f = a * x + b
    
    else:
        f = np.nan  # Caso não previsto
    
    return f

# Exemplo de uso:
# Assumindo que sel_pontos está definida em outro lugar
# T = pd.read_excel('seu_arquivo.xlsx')
# resultado = interpola(freq, press, T)
# ou
# valor_especifico = interpola(freq, press, T, indice=3)


# Exemplo de uso
if __name__ == "__main__":
    TabSimulador = pd.read_excel('../Dados/DoSimulador.xlsx')  # Carregue seus dados aqui
    TabSimulador = TabSimulador.iloc[1:]
    TabSimulador.columns = ['FreqBCSS', 'PressChegada', 'VazaoOleo', 'VazaoLiquido', 'Twh', 'Pwh', 'DeltaP']
    for coluna in TabSimulador.columns:
        TabSimulador[coluna] = TabSimulador[coluna].astype('float')
    TabSimulador['PressSuccao']=TabSimulador.Pwh-TabSimulador.DeltaP
    Freq = 50
    Press = 35
    Pontos = interpola(Freq, Press, TabSimulador, indice=3)
    print(Pontos)