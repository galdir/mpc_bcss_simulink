import pytest
import numpy as np
import pandas as pd
import casadi
from codigos_python.estima_vazao import interpola_casadi_vazao

def carregar_dados_excel(arquivo='..\..\..\Tabelas\DoSimulador.xlsx'):
    """
    Carrega os dados do arquivo Excel
    
    Args:
        arquivo: Nome do arquivo Excel
        
    Returns:
        numpy.ndarray: Matriz com as colunas [freq, press, vazao]
    """
    # Lê o arquivo Excel, pulando as duas primeiras linhas
    df = pd.read_excel(arquivo, skiprows=2, header=None)
    
    # Seleciona as três primeiras colunas
    matriz = df.iloc[:, :3].values
    
    return matriz

@pytest.fixture
def dados_excel():
    """
    Fixture que carrega os dados do arquivo Excel
    """
    try:
        return carregar_dados_excel()
    except FileNotFoundError:
        pytest.skip("Arquivo DoSimulador.xlsx não encontrado")

@pytest.fixture
def dados_teste():
    """
    Fixture que cria dados de teste para interpolação
    """
    # Criar grid de frequências
    freq = np.array([35, 40, 45])
    press = np.array([100, 150, 200])
    
    # Criar matriz de teste
    freq_grid, press_grid = np.meshgrid(freq, press)
    vazao = freq_grid * 2 - press_grid / 50  # Função exemplo para gerar vazões
    
    # Criar matriz de dados
    matriz = np.zeros((len(freq) * len(press), 3))
    matriz[:, 0] = freq_grid.flatten()
    matriz[:, 1] = press_grid.flatten()
    matriz[:, 2] = vazao.flatten()
    
    return {
        'matriz': matriz,
        'freq': freq,
        'press': press,
        'freq_grid': freq_grid,
        'press_grid': press_grid,
        'vazao': vazao
    }

@pytest.fixture
def funcao_interpolacao(dados_teste):
    """
    Fixture que cria a função de interpolação CasADi
    """
    freq_sym = casadi.MX.sym('freq')
    press_sym = casadi.MX.sym('press')
    
    return casadi.Function('interpolacao',
                         [freq_sym, press_sym],
                         [interpola_casadi_vazao(freq_sym, press_sym, dados_teste['matriz'])],
                         ['freq', 'press'],
                         ['vazao'])

@pytest.fixture
def funcao_interpolacao_excel(dados_excel):
    """
    Fixture que cria a função de interpolação CasADi usando dados do Excel
    """
    freq_sym = casadi.MX.sym('freq')
    press_sym = casadi.MX.sym('press')
    
    return casadi.Function('interpolacao',
                         [freq_sym, press_sym],
                         [interpola_casadi_vazao(freq_sym, press_sym, dados_excel)],
                         ['freq', 'press'],
                         ['vazao'])

def test_ponto_exato(funcao_interpolacao, dados_teste):
    """
    Testa interpolação em pontos exatos da grade
    """
    # Teste no primeiro ponto da grade
    freq_teste = dados_teste['freq'][0]
    press_teste = dados_teste['press'][0]
    
    vazao = funcao_interpolacao(freq_teste, press_teste)
    vazao_esperada = dados_teste['vazao'][0,0]
    
    assert np.abs(float(vazao) - vazao_esperada) < 1e-10, "Valor no ponto exato deve corresponder ao valor na grade"

def test_interpolacao_media(funcao_interpolacao):
    """
    Testa se a interpolação no ponto médio entre dois valores retorna a média
    """
    # Ponto médio entre dois valores conhecidos
    freq_teste = 37.5  # Meio do caminho entre 35 e 40
    press_teste = 125  # Meio do caminho entre 100 e 150
    
    vazao = float(funcao_interpolacao(freq_teste, press_teste))
    
    # O valor interpolado deve estar entre os valores extremos
    assert 60 < vazao < 80, "Valor interpolado deve estar dentro de limites razoáveis"

def test_limites_frequencia(funcao_interpolacao, dados_teste):
    """
    Testa o comportamento nos limites de frequência
    """
    # Teste abaixo do limite inferior
    freq_min = float(funcao_interpolacao(30, 150))
    freq_lim = float(funcao_interpolacao(dados_teste['freq'][0], 150))
    assert np.abs(freq_min - freq_lim) < 1e-10, "Deve limitar ao valor mínimo de frequência"
    
    # Teste acima do limite superior
    freq_max = float(funcao_interpolacao(50, 150))
    freq_lim = float(funcao_interpolacao(dados_teste['freq'][-1], 150))
    assert np.abs(freq_max - freq_lim) < 1e-10, "Deve limitar ao valor máximo de frequência"

def test_limites_pressao(funcao_interpolacao, dados_teste):
    """
    Testa o comportamento nos limites de pressão
    """
    # Teste abaixo do limite inferior
    press_min = float(funcao_interpolacao(40, 50))
    press_lim = float(funcao_interpolacao(40, dados_teste['press'][0]))
    assert np.abs(press_min - press_lim) < 1e-10, "Deve limitar ao valor mínimo de pressão"
    
    # Teste acima do limite superior
    press_max = float(funcao_interpolacao(40, 250))
    press_lim = float(funcao_interpolacao(40, dados_teste['press'][-1]))
    assert np.abs(press_max - press_lim) < 1e-10, "Deve limitar ao valor máximo de pressão"

def test_interpolacao_linear(funcao_interpolacao, dados_teste):
    """
    Testa interpolação linear quando um dos pontos é igual
    """
    # Teste com mesma frequência
    freq_teste = dados_teste['freq'][0]  # Frequência fixa
    press_teste = 125  # Ponto intermediário na pressão
    
    vazao = float(funcao_interpolacao(freq_teste, press_teste))
    
    vazao_inferior = float(funcao_interpolacao(freq_teste, dados_teste['press'][0]))
    vazao_superior = float(funcao_interpolacao(freq_teste, dados_teste['press'][1]))
    
    # O valor deve estar entre os valores conhecidos
    assert min(vazao_inferior, vazao_superior) <= vazao <= max(vazao_inferior, vazao_superior), \
        "Interpolação linear deve resultar em valor entre os extremos"

def test_monotonicidade(funcao_interpolacao):
    """
    Testa se a interpolação preserva a monotonicidade esperada
    """
    # Teste monotonicidade com frequência
    press_fixo = 150
    vazoes = [float(funcao_interpolacao(freq, press_fixo)) for freq in [35, 37, 40]]
    assert all(v1 < v2 for v1, v2 in zip(vazoes, vazoes[1:])), \
        "Vazão deve aumentar com o aumento da frequência"
    
    # Teste monotonicidade com pressão
    freq_fixo = 40
    vazoes = [float(funcao_interpolacao(freq_fixo, press)) for press in [100, 150, 200]]
    assert all(v1 > v2 for v1, v2 in zip(vazoes, vazoes[1:])), \
        "Vazão deve diminuir com o aumento da pressão"

def test_dimensoes_saida(funcao_interpolacao):
    """
    Testa se a saída tem as dimensões corretas
    """
    vazao = funcao_interpolacao(40, 150)
    assert vazao.shape == (1, 1), "Saída deve ser um escalar"


def test_dados_excel_pontos_especificos(funcao_interpolacao_excel, dados_excel):
    """
    Testa a interpolação usando dados reais do arquivo Excel
    """
    # Obtém valores únicos de frequência e pressão
    freqs = np.unique(dados_excel[:, 0])
    press = np.unique(dados_excel[:, 1])
    
    # Testa alguns pontos específicos
    pontos_teste = [
        (freqs[0], press[0]),          # Primeiro ponto
        (freqs[-1], press[-1]),        # Último ponto
        ((freqs[0] + freqs[1])/2, press[0]),  # Ponto intermediário na frequência
        (freqs[0], (press[0] + press[1])/2)   # Ponto intermediário na pressão
    ]
    
    for freq_teste, press_teste in pontos_teste:
        vazao = float(funcao_interpolacao_excel(freq_teste, press_teste))
        assert not np.isnan(vazao), f"Vazão não deve ser NaN para f={freq_teste}, p={press_teste}"
        assert vazao > 0, f"Vazão deve ser positiva para f={freq_teste}, p={press_teste}"

def test_dados_excel_monotonicidade(funcao_interpolacao_excel, dados_excel):
    """
    Testa a monotonicidade da interpolação usando dados reais do Excel
    """
    freqs = np.unique(dados_excel[:, 0])
    press = np.unique(dados_excel[:, 1])
    
    # Testa monotonicidade com frequência
    press_medio = np.median(press)
    freq_teste = np.linspace(freqs[0], freqs[-1], 10)
    vazoes = [float(funcao_interpolacao_excel(f, press_medio)) for f in freq_teste]
    
    # Verifica se a vazão geralmente aumenta com a frequência
    diferenca_vazao = np.diff(vazoes)
    assert np.mean(diferenca_vazao > 0) > 0.8, \
        "Vazão deve geralmente aumentar com o aumento da frequência"
    
    # Testa monotonicidade com pressão
    freq_media = np.median(freqs)
    press_teste = np.linspace(press[0], press[-1], 10)
    vazoes = [float(funcao_interpolacao_excel(freq_media, p)) for p in press_teste]
    
    # Verifica se a vazão geralmente diminui com a pressão
    diferenca_vazao = np.diff(vazoes)
    assert np.mean(diferenca_vazao < 0) > 0.8, \
        "Vazão deve geralmente diminuir com o aumento da pressão"

def test_dados_excel_limites(funcao_interpolacao_excel, dados_excel):
    """
    Testa o comportamento nos limites usando dados reais do Excel
    """
    freqs = np.unique(dados_excel[:, 0])
    press = np.unique(dados_excel[:, 1])
    
    # Testa limites de frequência
    freq_min = min(freqs) - 10
    freq_max = max(freqs) + 10
    press_med = np.median(press)
    
    vazao_abaixo = float(funcao_interpolacao_excel(freq_min, press_med))
    vazao_minima = float(funcao_interpolacao_excel(min(freqs), press_med))
    assert np.abs(vazao_abaixo - vazao_minima) < 1e-10, \
        "Deve limitar corretamente frequências abaixo do mínimo"
    
    vazao_acima = float(funcao_interpolacao_excel(freq_max, press_med))
    vazao_maxima = float(funcao_interpolacao_excel(max(freqs), press_med))
    assert np.abs(vazao_acima - vazao_maxima) < 1e-10, \
        "Deve limitar corretamente frequências acima do máximo"
    
    # Testa limites de pressão
    press_min = min(press) - 10
    press_max = max(press) + 10
    freq_med = np.median(freqs)
    
    vazao_abaixo = float(funcao_interpolacao_excel(freq_med, press_min))
    vazao_minima = float(funcao_interpolacao_excel(freq_med, min(press)))
    assert np.abs(vazao_abaixo - vazao_minima) < 1e-10, \
        "Deve limitar corretamente pressões abaixo do mínimo"
    
    vazao_acima = float(funcao_interpolacao_excel(freq_med, press_max))
    vazao_maxima = float(funcao_interpolacao_excel(freq_med, max(press)))
    assert np.abs(vazao_acima - vazao_maxima) < 1e-10, \
        "Deve limitar corretamente pressões acima do máximo"

def test_dados_excel_valores_intermediarios(funcao_interpolacao_excel, dados_excel):
    """
    Testa se os valores interpolados estão dentro dos limites esperados
    """
    freqs = np.unique(dados_excel[:, 0])
    press = np.unique(dados_excel[:, 1])
    
    # Testa pontos intermediários
    for i in range(len(freqs)-1):
        for j in range(len(press)-1):
            # Ponto intermediário
            freq_meio = (freqs[i] + freqs[i+1])/2
            press_meio = (press[j] + press[j+1])/2
            
            # Calcula vazão no ponto intermediário
            vazao = float(funcao_interpolacao_excel(freq_meio, press_meio))
            
            # Encontra valores nos pontos vizinhos
            vazoes_vizinhas = [
                float(funcao_interpolacao_excel(freqs[i], press[j])),
                float(funcao_interpolacao_excel(freqs[i+1], press[j])),
                float(funcao_interpolacao_excel(freqs[i], press[j+1])),
                float(funcao_interpolacao_excel(freqs[i+1], press[j+1]))
            ]
            
            # Verifica se o valor interpolado está entre os valores vizinhos
            assert min(vazoes_vizinhas) <= vazao <= max(vazoes_vizinhas), \
                f"Valor interpolado deve estar entre os valores vizinhos para f={freq_meio}, p={press_meio}"