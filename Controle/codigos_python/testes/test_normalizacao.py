import pytest
import numpy as np
from numpy.testing import assert_array_almost_equal
from codigos_python.normalizacao import (
    get_min_max_BCSS,
    normalizar_dado_BCS,
    desnormalizar_dado_BCS,
    normaliza_entradas,
    desnormaliza_predicoes
)

def test_get_min_max_BCSS():
    """Testa a função get_min_max_BCSS"""
    # Testa valores conhecidos
    assert get_min_max_BCSS('frequencia_BCSS') == (0, 60)
    assert get_min_max_BCSS('pressao_chegada') == (0, 100)
    
    # Testa erro para coluna inexistente
    with pytest.raises(KeyError):
        get_min_max_BCSS('coluna_inexistente')

def test_normalizar_dado_BCS():
    """Testa a função normalizar_dado_BCS"""
    # Testa valor único
    assert normalizar_dado_BCS(30, 'frequencia_BCSS') == 0.5  # (30 - 0)/(60 - 0) = 0.5
    assert normalizar_dado_BCS(50, 'pressao_chegada') == 0.5  # (50 - 0)/(100 - 0) = 0.5
    
    # Testa array
    valores = np.array([0, 30, 60])
    esperado = np.array([0.0, 0.5, 1.0])
    assert_array_almost_equal(normalizar_dado_BCS(valores, 'frequencia_BCSS'), esperado)
    
    # Testa erro para coluna inexistente
    with pytest.raises(KeyError):
        normalizar_dado_BCS(30, 'coluna_inexistente')

def test_desnormalizar_dado_BCS():
    """Testa a função desnormalizar_dado_BCS"""
    # Testa valor único
    assert desnormalizar_dado_BCS(0.5, 'frequencia_BCSS') == 30  # 0.5 * (60 - 0) + 0 = 30
    assert desnormalizar_dado_BCS(0.5, 'pressao_chegada') == 50  # 0.5 * (100 - 0) + 0 = 50
    
    # Testa array
    valores = np.array([0.0, 0.5, 1.0])
    esperado = np.array([0, 30, 60])
    assert_array_almost_equal(desnormalizar_dado_BCS(valores, 'frequencia_BCSS'), esperado)
    
    # Testa erro para coluna inexistente
    with pytest.raises(KeyError):
        desnormalizar_dado_BCS(0.5, 'coluna_inexistente')

def test_normaliza_entradas():
    """Testa a função normaliza_entradas"""
    # Cria entrada de teste
    entrada = np.array([
        30,    # frequencia (max=60)
        50,    # pressao_montante (max=100)
        125,   # pressao_succao (max=250)
        50,    # pressao_chegada (max=100)
        100,   # pressao_diferencial (max=200)
        150,   # pressao_descarga (max=300)
        100,   # temperatura_motor (max=200)
        100,   # corrente_torque (max=200)
        100,   # corrente_total (max=200)
        125,   # temperatura_succao (max=250)
        50,    # vibracao (max=100)
        50     # temperatura_chegada (max=100)
    ])
    
    # Valores esperados normalizados
    esperado = np.array([
        0.5,  # 30/60
        0.5,  # 50/100
        0.5,  # 125/250
        0.5,  # 50/100
        0.5,  # 100/200
        0.5,  # 150/300
        0.5,  # 100/200
        0.5,  # 100/200
        0.5,  # 100/200
        0.5,  # 125/250
        0.5,  # 50/100
        0.5   # 50/100
    ])
    
    assert_array_almost_equal(normaliza_entradas(entrada), esperado)
    
    # Testa erro para array de tamanho incorreto
    with pytest.raises(IndexError):
        normaliza_entradas(np.array([1, 2, 3]))

def test_desnormaliza_predicoes():
    """Testa a função desnormaliza_predicoes"""
    # Cria entrada normalizada de teste
    entrada_norm = np.array([
        0.5,  # pressao_succao (max=250)
        0.5,  # pressao_chegada (max=100)
        0.5,  # pressao_diferencial (max=200)
        0.5,  # pressao_descarga (max=300)
        0.5,  # temperatura_motor (max=200)
        0.5,  # corrente_torque (max=200)
        0.5,  # corrente_total (max=200)
        0.5,  # temperatura_succao (max=250)
        0.5,  # vibracao (max=100)
        0.5   # temperatura_chegada (max=100)
    ])
    
    # Valores esperados desnormalizados
    esperado = np.array([
        125,  # 0.5 * 250
        50,   # 0.5 * 100
        100,  # 0.5 * 200
        150,  # 0.5 * 300
        100,  # 0.5 * 200
        100,  # 0.5 * 200
        100,  # 0.5 * 200
        125,  # 0.5 * 250
        50,   # 0.5 * 100
        50    # 0.5 * 100
    ])
    
    assert_array_almost_equal(desnormaliza_predicoes(entrada_norm), esperado)
    
    # Testa erro para array de tamanho incorreto
    with pytest.raises(IndexError):
        desnormaliza_predicoes(np.array([1, 2, 3]))

def test_ciclo_normalizacao_desnormalizacao():
    """Testa ciclo completo de normalização e desnormalização"""
    # Testa para cada coluna
    colunas = [
        'frequencia_BCSS',
        'pressao_chegada',
        'pressao_succao_BCSS',
        'pressao_diferencial_BCSS',
        'pressao_descarga_BCSS'
    ]
    
    for coluna in colunas:
        valor_original = 30
        valor_normalizado = normalizar_dado_BCS(valor_original, coluna)
        valor_final = desnormalizar_dado_BCS(valor_normalizado, coluna)
        assert abs(valor_original - valor_final) < 1e-10