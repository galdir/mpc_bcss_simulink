import pytest
import numpy as np
import casadi
from codigos_python.estima_vazao import sel_pontos_casadi_vazao


@pytest.fixture
def dados_teste():
    """
    Fixture que cria dados de teste para ser usado por múltiplos testes
    """
    # Criar grid de frequências (35 a 65 Hz, passo de 5)
    freqs = np.arange(35, 70, 5)
    
    # Criar grid de pressões (100 a 300 bar, passo de 50)
    press = np.arange(100, 350, 50)
    
    # Criar todas as combinações de frequência e pressão
    freq_grid, press_grid = np.meshgrid(freqs, press)
    
    # Criar vazões simuladas
    vazao = freq_grid * 2 - press_grid / 50
    
    # Criar matriz com todos os dados
    matriz = np.zeros((len(freqs) * len(press), 3))
    matriz[:, 0] = freq_grid.flatten()  # Frequências
    matriz[:, 1] = press_grid.flatten() # Pressões
    matriz[:, 2] = vazao.flatten()      # Vazões
    
    return {
        'matriz': matriz,
        'freqs': freqs,
        'press': press,
        'freq_grid': freq_grid,
        'press_grid': press_grid,
        'vazao': vazao
    }

@pytest.fixture
def funcao_casadi(dados_teste):
    """
    Fixture que cria a função CasADi para ser usada nos testes
    """
    freq = casadi.MX.sym('freq')
    press = casadi.MX.sym('press')
    
    return casadi.Function('pontos', 
                         [freq, press],
                         [sel_pontos_casadi_vazao(freq, press, dados_teste['matriz'])['vazao_oleo'],
                          sel_pontos_casadi_vazao(freq, press, dados_teste['matriz'])['frequencia_BCSS'],
                          sel_pontos_casadi_vazao(freq, press, dados_teste['matriz'])['pressao_chegada']],
                         ['freq', 'press'],
                         ['vazao', 'freq_sel', 'press_sel'])

def test_pontos_dentro_limites(funcao_casadi):
    """
    Testa seleção de pontos para valores dentro dos limites
    """
    freq_teste, press_teste = 40, 150
    vazao, freq_sel, press_sel = funcao_casadi(freq_teste, press_teste)
    
    # Converte objetos DM para arrays numpy
    freq_sel_np = np.array(freq_sel.full()).flatten()
    press_sel_np = np.array(press_sel.full()).flatten()
    
    # Verifica se os pontos selecionados formam um retângulo válido
    assert len(freq_sel_np) == 4, "Deve retornar 4 pontos"
    assert len(np.unique(freq_sel_np)) <= 2, "Deve usar no máximo 2 frequências diferentes"
    assert len(np.unique(press_sel_np)) <= 2, "Deve usar no máximo 2 pressões diferentes"
    
    # Verifica se os pontos envolvem o ponto de teste
    if len(np.unique(freq_sel_np)) == 2:  # Se não está exatamente em um ponto da grade
        assert np.min(freq_sel_np) <= freq_teste <= np.max(freq_sel_np)
    if len(np.unique(press_sel_np)) == 2:
        assert np.min(press_sel_np) <= press_teste <= np.max(press_sel_np)

def test_limites_frequencia(funcao_casadi, dados_teste):
    """
    Testa o comportamento quando a frequência está fora dos limites
    """
    # Teste abaixo do limite
    freq_teste, press_teste = 30, 200
    vazao, freq_sel, press_sel = funcao_casadi(freq_teste, press_teste)
    freq_sel_np = np.array(freq_sel.full()).flatten()
    assert np.all(freq_sel_np >= min(dados_teste['freqs']))
    
    # Teste acima do limite
    freq_teste, press_teste = 75, 200
    vazao, freq_sel, press_sel = funcao_casadi(freq_teste, press_teste)
    freq_sel_np = np.array(freq_sel.full()).flatten()
    assert np.all(freq_sel_np <= max(dados_teste['freqs']))

def test_limites_pressao(funcao_casadi, dados_teste):
    """
    Testa o comportamento quando a pressão está fora dos limites
    """
    # Teste abaixo do limite
    freq_teste, press_teste = 50, 50
    vazao, freq_sel, press_sel = funcao_casadi(freq_teste, press_teste)
    press_sel_np = np.array(press_sel.full()).flatten()
    assert np.all(press_sel_np >= min(dados_teste['press']))
    
    # Teste acima do limite
    freq_teste, press_teste = 50, 400
    vazao, freq_sel, press_sel = funcao_casadi(freq_teste, press_teste)
    press_sel_np = np.array(press_sel.full()).flatten()
    assert np.all(press_sel_np <= max(dados_teste['press']))

def test_ponto_exato_grade(funcao_casadi, dados_teste):
    """
    Testa quando o ponto está exatamente em um ponto da grade
    """
    freq_teste, press_teste = 35, 100  # Ponto exato da grade
    vazao, freq_sel, press_sel = funcao_casadi(freq_teste, press_teste)
    
    # Converte para numpy arrays
    freq_sel_np = np.array(freq_sel.full()).flatten()
    press_sel_np = np.array(press_sel.full()).flatten()
    
    # Verifica se todos os pontos são iguais quando estamos exatamente em um ponto da grade
    assert len(np.unique(freq_sel_np)) == 1, "Todas as frequências devem ser iguais"
    assert len(np.unique(press_sel_np)) == 1, "Todas as pressões devem ser iguais"
    assert np.abs(freq_sel_np[0] - freq_teste) < 1e-10, "Frequência selecionada deve ser igual à de teste"
    assert np.abs(press_sel_np[0] - press_teste) < 1e-10, "Pressão selecionada deve ser igual à de teste"

def test_consistencia_valores_vazao(funcao_casadi, dados_teste):
    """
    Testa se os valores de vazão são consistentes com a matriz original
    """
    # Testa um ponto específico da grade
    freq_idx = 0  # Primeiro ponto de frequência
    press_idx = 0  # Primeiro ponto de pressão
    
    freq_teste = dados_teste['freqs'][freq_idx]
    press_teste = dados_teste['press'][press_idx]
    
    vazao, freq_sel, press_sel = funcao_casadi(freq_teste, press_teste)
    vazao_np = np.array(vazao.full()).flatten()
    
    # O valor de vazão deve corresponder ao valor na matriz original
    valor_esperado = dados_teste['vazao'][press_idx, freq_idx]
    assert np.abs(vazao_np[0] - valor_esperado) < 1e-10, "Valor de vazão deve corresponder à matriz original"

def test_dimensoes_saida(funcao_casadi):
    """
    Testa se as dimensões das saídas estão corretas
    """
    freq_teste, press_teste = 45, 200
    vazao, freq_sel, press_sel = funcao_casadi(freq_teste, press_teste)
    
    # Converte para numpy arrays e verifica dimensões
    vazao_np = np.array(vazao.full()).flatten()
    freq_sel_np = np.array(freq_sel.full()).flatten()
    press_sel_np = np.array(press_sel.full()).flatten()
    
    assert len(vazao_np) == 4, "Deve retornar 4 valores de vazão"
    assert len(freq_sel_np) == 4, "Deve retornar 4 valores de frequência"
    assert len(press_sel_np) == 4, "Deve retornar 4 valores de pressão"