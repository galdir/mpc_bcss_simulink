import pandas as pd
import numpy as np

def avalia_condicao(T, BTP):
    """
    Recebe os DataFrames T e BTP. Com base nisso,
    reproduz as contas para análise das condições de Up e Downthrust
    conforme contas do Anexo C1.

    :param T: DataFrame com os dados de entrada
    :param BTP: DataFrame com as condições do BTP
    :return: DataFrame com Qdt, Qut, Condicao, Cor
    """
    # Configurações para mostrar resultados (útil para depuração)
    mostra_memoria = False
    mostra_resultados = False

    resultados = []
    


    for i in range(len(T)):
        # Assumindo que BTP tem apenas uma linha. Se tiver múltiplas, ajuste conforme necessário.
        btp = BTP.iloc[0]

        # Contas que reproduzem a MEMÓRIA DE CÁLCULO do Anexo C1
        QL = T['VazaoOleo'].iloc[i] / (100 - btp['BSW']) * 100
        do = 141.5 / (131.5 + btp['API'])
        Ppc = 708.75 - 57.5 * btp['dg']
        Tpc = 169 + 314 * btp['dg']
        Psuc = (T['PressSuccao'].iloc[i] / 1.01972) * 14.50377 + 14.7
        Tsuc = (T['Twh'].iloc[i] + 273.15) * 9/5
        RGO = btp['RGO'] / 0.1781
        Pb = btp['Pb'] * 14.22334 + 14.7

        if Psuc <= Pb:
            Rs1 = btp['dg'] * ((Psuc / 18.2 + 1.4) * 10**(0.0125 * btp['API'] - 0.00091 * (Tsuc - 459.67)))**1.2048
        else:
            Rs1 = RGO

        Rs2 = Rs1 * 0.1781
        Bo = 0.972 + 0.000147 * (Rs1 * (btp['dg'] / do)**0.5 + 1.25 * (Tsuc - 460))**1.175
        Ppr = Psuc / Ppc
        Tpr = Tsuc / Tpc
        z = 1 - 3.52 * Ppr / (10**(0.9813 * Tpr)) + 0.274 * Ppr**2 / (10**(0.8157 * Tpr))
        Bg = (14.7 / Psuc) * (Tsuc * z) / 520
        Bw = (1 + (-1.0001e-2 + 1.33391e-4 * (Tsuc - 459.67) + 5.50654e-7 * (Tsuc - 459.67))) * \
             (1 + (-1.95301e-9 * Psuc * (Tsuc - 459.67) - 1.72834e-13 * (Psuc**2) * (Tsuc - 459.67) - 
                   3.58922e-7 * Psuc - 2.25341e-10 * (Psuc**2)))

        # Contas que reproduzem os RESULTADOS do Anexo C1
        QoPT = T['VazaoOleo'].iloc[i] * Bo
        QwPT = QL * (btp['BSW'] / 100) * Bw
        QgPT = (btp['RGO'] - Ppr) * T['VazaoOleo'].iloc[i] * Bg
        Qtotal = QoPT + QwPT + QgPT
        FGL = QgPT / Qtotal * 100
        Qdt = 3021 * T['FreqBCSS'].iloc[i] / 60
        Qut = 5564 * T['FreqBCSS'].iloc[i] / 60

        # Verifica condição de vazão para Up e Downthrust
        if Qtotal >= Qut:
            Condicao = "Upthrust"
            Cor = 0
        elif Qtotal <= Qdt:
            Condicao = "Downthrust"
            Cor = 1
        else:
            Condicao = "Normal"
            Cor = 2

        resultados.append({
            'Qdt': Qdt,
            'Qut': Qut,
            'Condicao': Condicao,
            'Cor': Cor
        })

        # Mostra resultados se configurado
        if mostra_memoria:
            print(f"Memória de Cálculo para registro {i+1}:")
            # Adicione aqui os prints desejados para a memória de cálculo

        if mostra_resultados:
            print(f"Resultados para registro {i+1}:")
            # Adicione aqui os prints desejados para os resultados

    return pd.DataFrame(resultados)

# Exemplo de uso:
# T = pd.read_excel('seu_arquivo_T.xlsx')
# BTP = pd.read_excel('seu_arquivo_BTP.xlsx')
# resultados = avalia_condicao(T, BTP)


# Exemplo de uso
if __name__ == "__main__":
    TabSimulador = pd.read_excel('../Dados/DoSimulador.xlsx')  # Carregue seus dados aqui
    TabSimulador = TabSimulador.iloc[1:]
    TabSimulador.columns = ['FreqBCSS', 'PressChegada', 'VazaoOleo', 'VazaoLiquido', 'Twh', 'Pwh', 'DeltaP']
    for coluna in TabSimulador.columns:
        TabSimulador[coluna] = TabSimulador[coluna].astype('float')
    TabSimulador['PressSuccao']=TabSimulador.Pwh-TabSimulador.DeltaP
    BTP = pd.read_excel('../Dados/DoBTP.xlsx')
    BTP = BTP.iloc[1:]
    resultados = avalia_condicao(TabSimulador, BTP)
    print(resultados)