import numpy as np

from normalizacao import desnormaliza_predicoes, normaliza_entradas


def executa_predicao(EntradaU, EstadosX, ESNdataa0, modelo_ESN, f_matrizVazao_sym):
    """
    Executa a predição combinando ESN e tabela de vazão
    
    Parâmetros:
    EntradaU -- entradas do processo (frequencia_aplicada, pressao_montante_alvo_aplicada)
    EstadosX -- 11 variáveis do processo no instante atual
    ESNdataa0 -- estado atual do reservatório
    modelo_ESN -- modelo contendo Wrr, Wir, Wbr
    f_matrizVazao_sym -- função que retorna vazão estimada
    
    Retorna:
    predicoes -- vetor com predições dos novos estados
    novo_a0 -- novo estado do reservatório
    """
    conversao_bar_kgf = 1.019716
    
    # Executa predição da ESN
    predicoes, novo_a0 = executa_predicao_ESN(EntradaU, EstadosX, ESNdataa0, modelo_ESN)
    
    # Executa predição da vazão
    Freq = EntradaU[0]
    PChegada = predicoes[1] * conversao_bar_kgf  # Converte pressão de bar para Kgf/cm2
    vazaoOleo_estimada = f_matrizVazao_sym(Freq, PChegada)
    
    # Monta vetor final incluindo a vazão estimada
    predicoes = np.append(predicoes, vazaoOleo_estimada)
    
    return predicoes, novo_a0

def executa_predicao_ESN(EntradaU, EstadosX, ESNdataa0, modelo_ESN):
    """
    Executa a predição da ESN um passo a frente
    
    Parâmetros:
    EntradaU -- entradas do processo no instante atual
    EstadosX -- variáveis do processo no instante atual
    ESNdataa0 -- estado atual do reservatório da ESN
    modelo_ESN -- modelo contendo Wrr, Wir, Wbr
    
    Retorna:
    predicoes -- vetor com estimativas das variáveis do processo
    novo_a0 -- novos estados do reservatório da ESN
    """
    # Monta vetor de entrada (excluindo vazão)
    entradas = np.concatenate([EntradaU, EstadosX[:-1]])
    
    # Normaliza entradas
    entradas_normalizadas = normaliza_entradas(entradas)
    
    # Calcula novo estado do reservatório
    x_ESN = (modelo_ESN.data['Wrr'] @ ESNdataa0 + 
             modelo_ESN.data['Wir'] @ entradas_normalizadas + 
             modelo_ESN.data['Wbr'])
    
    # Atualiza estado da ESN
    novo_a0 = ((1 - modelo_ESN.data['gama']) * ESNdataa0 + 
               modelo_ESN.data['gama'] * np.tanh(x_ESN))
    
    # Adiciona bias
    a_wbias = np.concatenate([[1.0], novo_a0])
    
    # Calcula predições
    predicoes_normalizadas = modelo_ESN.data['Wro'] @ a_wbias
    
    # Desnormaliza as predições
    predicoes = desnormaliza_predicoes(predicoes_normalizadas)
    
    return predicoes, novo_a0


# Exemplo de uso
if __name__ == "__main__":
    import scipy.io as sio

    # Nome do arquivo .mat a ser carregado
    nome_esn = 'weightsESNx_TR400_TVaz0.9_RaioE0.4.mat'
    

    # Carrega o arquivo .mat
    modelo_preditor = sio.loadmat(nome_esn)