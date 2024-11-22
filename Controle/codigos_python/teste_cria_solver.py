import scipy.io as sio
from pathlib import Path
import pandas as pd
import casadi as ca
import numpy as np
import time

from cria_solver import busca_condicao_inicial, cria_funcao_h, cria_solver
from limites import cria_busca_limites_casadi
from predicao import esquentar_esn
from estima_vazao import cria_estimador_vazao_casadi

conversao_bar_kgf = 1.019716

# Nome do arquivo .mat a ser carregado
nome_esn = 'weightsESNx_TR400_TVaz0.9_RaioE0.4.mat'
caminho_modelos = Path("./Modelos/ESN/")
caminho_modelo_esn = caminho_modelos / nome_esn

arquivo_matriz_vazao = 'DoSimulador.xlsx'
caminho_tabelas = Path("./Tabelas")
df_matriz_vazao = pd.read_excel(
    caminho_tabelas / arquivo_matriz_vazao, skiprows=2, header=None)
matriz = df_matriz_vazao.iloc[:, :3].values
# Carrega o arquivo .mat
modelo_preditor = sio.loadmat(caminho_modelo_esn)

estimador_vazao = cria_estimador_vazao_casadi(matriz)

pasta_dados = Path(
    'C:\\petrobras_2023_sistema_controle_inteligente_operacao_BCS_campo\\DADOS UTEIS\\dados MPA OPC\\')
nome_arquivo_dados = 'df_opc_mpa_10s.parquet'

df = pd.read_parquet(pasta_dados / nome_arquivo_dados)

data_hora = '2024-07-17 01:00:00'

x_atual, u_atual = busca_condicao_inicial(data_hora, estimador_vazao, df)

variaveis_manipuladas = ['frequencia_BCSS',
                         'pressao_montante_alvo']

variaveis_processo = ['pressao_succao_BCSS',
                      'pressao_chegada',
                      'pressao_diferencial_BCSS',
                      'pressao_descarga_BCSS',
                      'temperatura_motor_BCSS',
                      'corrente_torque_BCSS',
                      'corrente_total_BCSS',
                      'temperatura_succao_BCSS',
                      'vibracao_BCSS',
                      'temperatura_chegada',
                      'vazao_oleo']

variaveis_preditoras = variaveis_manipuladas + variaveis_processo

reservatorio_esn = modelo_preditor['data']['a0'][0][0].reshape(-1)

entradas = u_atual + x_atual

# esquentar ESN
reservatorio_esn = esquentar_esn(
    entradas, reservatorio_esn, modelo_preditor, estimador_vazao)

# Limites máx/min para ser dado pelo controlador como entrada de Freq no processo
freq_max_min = [60,  40]
# Limites máx/min para ser dado pelo controlador como entrada de PMon no processo
p_mon_alvo_max_min = [50, 20]
# Vetor com valor máximo das manipuladas (Freq e PMonAlvo)
umax = [freq_max_min[0],  p_mon_alvo_max_min[0]]
# Vetor com valor mínimo das manipuladas  (Freq e PMonAlvo)
umin = [freq_max_min[1], p_mon_alvo_max_min[1]]
# Variação máxima nas manipuladas [ Hz    bar ]
dumax = [0.1, 1]
margem_percentual = 1

hp = 3
hc = hp-1

# Qy - Peso das saidas controladas por setpoint = PChegada e Vazao)
matriz_qy = np.diag([1,  10])
# Qu - Peso das ações de controle nas entradas (Alvos Desejados em  Freq. e PMonAlvo)
matriz_qu = np.diag([10,  1])
# Peso para os erros de estimação das  variáveis do processo
matriz_qx = 0 * np.diag(np.ones(11))
# R - Peso na variação das ações de controle - DeltaU em Freq. e PMonAlvo
matriz_r = 0 * np.diag([1,  1])

# Número de variáveis (estados) do processo
nx = matriz_qx.shape[0]

# Número de variáveis de entrada no processo (manipuladas)
nu = matriz_qu.shape[0]

# Número de variáveis de saida controladas por SetPoint
ny = matriz_qy.shape[0]

funcao_h = cria_funcao_h(nx)

wall_time = 10  # Tempo máximo de execução

# Tabela completa com pré-cálculos dos limites dinâmicos em função da frequência
arquivo_limites_integrados = 'TabelaLimitesDinamicos.xlsx'
caminho_tabelas = Path("./Tabelas")
df_limites_integrados = pd.read_excel(
    caminho_tabelas / arquivo_limites_integrados)
matriz_limites_integrados = df_limites_integrados.values

busca_limites = cria_busca_limites_casadi(matriz_limites_integrados)

solver, args = cria_solver(umax, umin, dumax, margem_percentual,
                           hp, hc, matriz_qy, matriz_qu, matriz_r, matriz_qx, nx, nu, ny,
                           estimador_vazao, busca_limites, modelo_preditor, funcao_h, wall_time)

# x0 = XIni
# Condição inicial das variáveis medidas (estados X)
x_atual_hp = np.tile(x_atual, (1 + hp))
# u0 = UIni
# Condição inicial das ações de controle (U)
u_atual_hp = np.tile(u_atual, (hp))

freq_alvo_atual = 60
limites = busca_limites(freq_alvo_atual).full()
# Mais conservador entre limite minimo e PMonAlvoMin
p_mon_alvo_atual = max(limites[1, 1], p_mon_alvo_max_min[1])
alvos_eng = np.array([freq_alvo_atual, p_mon_alvo_atual])

vazao_alvo_ini = float(estimador_vazao(
    alvos_eng[0], alvos_eng[1] * conversao_bar_kgf))

ysp = np.hstack([
    alvos_eng[1],
    vazao_alvo_ini
])

erro_x = np.zeros((nx))
erro_y = np.zeros((ny))
buff_delta_freq = np.zeros(45)
# Inicializa valores futuros (variáveis de decisão)
du_atual_hp = np.zeros((hp * nu))

solucao_anterior = np.hstack([x_atual_hp, u_atual_hp, du_atual_hp])

args['p'] = np.hstack([x_atual, u_atual, alvos_eng, ysp, erro_x,
                       erro_y, buff_delta_freq, reservatorio_esn])

t1_start = time.process_time() 

solucao = solver(
    x0=solucao_anterior,
    lbx=args['lbx'],
    ubx=args['ubx'],
    lbg=args['lbg'],
    ubg=args['ubg'],
    p=args['p']
)

t1_stop =  time.process_time()
print("Elapsed time during the whole program in seconds:",
                                         t1_stop-t1_start) 

# solucao = solver(
#     x0=ca.MX(solucao_anterior),
#     lbx=ca.MX(args['lbx']),
#     ubx=ca.MX(args['ubx']),
#     lbg=ca.MX(args['lbg']),
#     ubg=ca.MX(args['ubg']),
#     p=ca.MX(args['p'])
# )

feasible = solver.stats()['success']
iteracoes = solver.stats()['iter_count']

print('feasible:')
print(feasible)
print('iteracoes:')
print(iteracoes)
#print(f"Constraint violations: {solver.stats()['g_violation']}")
stats = solver.stats()
total_wall_time = (stats['t_wall_nlp_f'] + stats['t_wall_nlp_g'] + 
                  stats['t_wall_nlp_grad_f'] + stats['t_wall_nlp_jac_g'] + 
                  stats['t_wall_nlp_hess_l'])
print(f'tempo solver: {total_wall_time}')

solucao_manipuladas = solucao["x"].full().reshape(-1)[(-hp*3*2):]

print(f'solucao: {solucao["x"].full().reshape(-1)}')