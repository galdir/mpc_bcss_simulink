import casadi
import numpy as np

from limites import cria_busca_limites_casadi
from estima_vazao import cria_estimador_vazao_casadi
from predicao import esquentar_esn, executa_predicao_casadi


def cria_solver(umax, umin, dumax, margem_percentual,
                hp, hc, matriz_qy, matriz_qu, matriz_r, matriz_qx, nx, nu, ny,
                estima_vazao, busca_limites, modelo_preditor, funcao_h, walltime):
    """
    Cria um solver MPC usando CasADi.

    Mantém os mesmos parâmetros da versão MATLAB, preservando a funcionalidade
    e nomenclatura em português.
    """

    dumin = [-x for x in dumax] 
    
    # Inicialização do timer e importação do CasADi
    nx_ESN = len(modelo_preditor['data']['a0'][0][0])

    # Variáveis simbólicas para o problema de otimização
    X = casadi.MX.sym('X', nx, 1+hp)  # Estado atual + Estados futuros até Hp
    U = casadi.MX.sym('U', nu, hp)    # Ações de controle até o horizonte Hp
    # Variações nas ações de controle sobre o horizonte Hp
    DU = casadi.MX.sym('DU', nu, hp)

    # Parâmetros que foram oferecidos para o Solver
    indice_parametros = [
        nx,     # qtd Medições
        nu,     # qtd Ações
        nu,     # qtd AlvoEng
        ny,     # qtd Ysp
        nx,     # qtd ErroX
        ny,     # qtd ErroY
        45,     # tamanho do Buffer de DeltaFreq
        nx_ESN  # tamanho do estado do modelo de predicao
    ]

    # Cria vetor de parâmetros na dimensão especificada
    parametros = casadi.MX.sym('P', sum(indice_parametros))

    # Associa variáveis simbólicas as respectivas partes no vetor de parâmetros
    xk0, uk0, alvo_eng, ysp, erro_x, erro_y, buff_delta_freq, reservatorio_esn = extrai_parametros(
        parametros, indice_parametros)

    modelo_preditor_estado_atual = reservatorio_esn

    # Inicializa variável que vai armazenar a estrutura de argumentos do NLP
    args = {
        'lbx': [],  # Limites inferiores para as restrições dos estados do MPC
        'ubx': [],  # Limites superiores para as restrições dos estados do MPC
    }

    # Para os estados atuais e para todo o horizonte Hp
    for k in range(1+hp):
        args['lbx'].extend([0] * nx)
        args['ubx'].extend([float('inf')] * nx)

    # Para as ações de controle em todo o horizonte futuro
    for k in range(hp):
        args['lbx'].append(umin)
        args['ubx'].append(umax)

    # Restrições para os limites na variação das ações de controle
    for k in range(hp):
        args['lbx'].append(dumin)
        args['ubx'].append(dumax)

    # Montando as restrições de igualdade/desigualdade em g
    g = []
    args['lbg'] = []
    args['ubg'] = []

    # Multipleshooting = restrições de igualdade para seguir dinâmica do sistema
    g.append(X[:, 0] - xk0)
    args['lbg'].extend([0] * nx)
    args['ubg'].extend([0] * nx)

    # Limita a primeira ação de controle pelo máximo DeltaU
    g.append(U[:, 0] - uk0 - DU[:, 0])
    args['lbg'].extend([0] * nu)
    args['ubg'].extend([0] * nu)

    # Restrições de igualdade para assegurar que os estados futuros vão seguir as predições
    for k in range(hp):
        x_predito, modelo_preditor_novo_estado = executa_predicao_casadi(
            U[:, k], X[:, k], modelo_preditor_estado_atual, modelo_preditor, estima_vazao)
        modelo_preditor_estado_atual = modelo_preditor_novo_estado

        g.append(X[:, k+1] - x_predito)
        args['lbg'].extend([0] * nx)
        args['ubg'].extend([0] * nx)

    # Restrições de igualdade para definir DeltaU em função de U
    for k in range(hp-1):
        Soma = U[:, k+1] - U[:, k] - DU[:, k+1]
        g.append(Soma)
        args['lbg'].extend([0] * nu)
        args['ubg'].extend([0] * nu)

        # Restrição avaliando as variações acumuladas na frequencia
        buff_delta_freq = casadi.vertcat(DU[0, k], buff_delta_freq[:-1])
        Soma = casadi.sum1(buff_delta_freq)

        g.append(Soma)
        args['lbg'].append(-1)
        args['ubg'].append(1)

    # Restrições dinâmicas para os estados X e para as saidas Y
    for k in range(hp):
        LimitesX = busca_limites(U[0, k]).T
        LimitesX[:, 0] *= (1 - margem_percentual/100)
        LimitesX[:, 1] *= (1 + margem_percentual/100)

        LimitesY = funcao_h(LimitesX)

        # Restrições para as variáveis do processo (estados X)
        LimMaxX = LimitesX[:, 0] - X[:, k+1]
        g.append(LimMaxX)
        args['lbg'].extend([0] * nx)
        args['ubg'].extend([float('inf')] * nx)

        LimMinX = X[:, k+1] - LimitesX[:, 1]
        g.append(LimMinX)
        args['lbg'].extend([0] * nx)
        args['ubg'].extend([float('inf')] * nx)

        # Restrições para as variáveis de saida
        y_saida = funcao_h(X[:, k+1])

        LimMaxY = LimitesY[:, 0] - y_saida
        g.append(LimMaxY)
        args['lbg'].extend([0] * ny)
        args['ubg'].extend([float('inf')] * ny)

        LimMinY = y_saida - LimitesY[:, 1]
        g.append(LimMinY)
        args['lbg'].extend([0] * ny)
        args['ubg'].extend([float('inf')] * ny)

        # Limites PMonAlvo
        ValMin = max(LimitesX[1, 1], umin[1])
        DiferencaMin = U[1, k] - ValMin
        g.append(DiferencaMin)
        args['lbg'].append(0)
        args['ubg'].append(float('inf'))

        ValMax = min(LimitesX[1, 0], umax[1])
        DiferencaMax = ValMax - U[1, k]
        g.append(DiferencaMax)
        args['lbg'].append(0)
        args['ubg'].append(float('inf'))

    # Depois do instante Hc, seguir a teoria e manter a mesma ação de controle futura
    for k in range(hc, hp):
        g.append(U[:, k] - U[:, k-1])
        args['lbg'].extend([0] * nu)
        args['ubg'].extend([0] * nu)

    # Preparando o custo da função objetivo
    fob = 0
    fob += casadi.mtimes(casadi.mtimes(erro_x.T, matriz_qx), erro_x)

    for k in range(hp):
        y_saida = funcao_h(X[:, k+1])
        erro_termo = y_saida - ysp + erro_y
        fob += casadi.mtimes(casadi.mtimes(erro_termo.T, matriz_qy), erro_termo)

    for k in range(hc):
        erro_controle = U[:, k] - alvo_eng
        fob += casadi.mtimes(casadi.mtimes(erro_controle.T, matriz_qu), erro_controle)
        fob += casadi.mtimes(casadi.mtimes(DU[:, k].T, matriz_r), DU[:, k])

    # Monta as variáveis de decisão em um vetor coluna
    variaveis_opt = casadi.vertcat(
        casadi.vec(X),
        casadi.vec(U),
        casadi.vec(DU)
    )

    # Define a estrutura para problema de otimização não linear
    nlp = {
        'f': fob,
        'x': variaveis_opt,
        'g': casadi.vertcat(*g),
        'p': parametros
    }

    # Configuração específica do IPOPT
    opcoes = {
        'print_time': 0,
        'ipopt': {
            'print_level': 0,
            'bound_relax_factor': 0,
            'max_iter': 1000,
            'max_wall_time': walltime
        },
        'verbose': 0
    }

    # Define o solver IPOPT para resolver o problema de otimização não linear
    solver = casadi.nlpsol('solver', 'ipopt', nlp, opcoes)

    return solver, args


def extrai_parametros(P, Indice):
    """
    Extrai as partes que compõe os parâmetros do Solver.

    Similar à função ExtraiParametros do MATLAB.
    """
    soma_acumulada = np.cumsum([0] + Indice)
    fatias = [slice(inicio, fim) for inicio, fim in zip(
        soma_acumulada[:-1], soma_acumulada[1:])]

    return [P[s] for s in fatias]


def busca_condicao_inicial(data_hora, estimador_vazao, df):

    # Retorna condição inicial na data_hora pré-selecionada para inicio da simulação do controle. #
    # Inclui as condilçoes iniciais das variáveis do processo (XIni) e das variáveis manipuladas (UIni)

    conversao_bar_kgf = 1.019716

    # Carrega dados para condição inicial da planta na faixa de datas pré-selecionadas
    df = df.query('index >= @data_hora').copy()

    # Atribui a coniição inicial na data especificada
    # Para os valores das entradas
    u_ini = [df.frequencia_BCSS.iloc[0],
             df.pressao_montante_alvo.iloc[0]]

    # Para as variáveis do processo
    # Entra Freq[Hz] e PChegada[Kgf/cm2] para retornar a vazão estimada em m3/dia
    vazao_ini = float(estimador_vazao(
        u_ini[0], df.pressao_chegada.iloc[0] * conversao_bar_kgf))

    x_ini = [df.pressao_succao_BCSS.iloc[0],
             df.pressao_chegada.iloc[0],
             df.pressao_diferencial_BCSS.iloc[0],
             df.pressao_descarga_BCSS.iloc[0],
             df.temperatura_motor_BCSS.iloc[0],
             df.corrente_torque_BCSS.iloc[0],
             df.corrente_total_BCSS.iloc[0],
             df.temperatura_succao_BCSS.iloc[0],
             df.vibracao_BCSS.iloc[0],
             df.temperatura_chegada.iloc[0],
             vazao_ini]

    return [x_ini, u_ini]


# Exemplo de uso
if __name__ == "__main__":
    import scipy.io as sio
    from pathlib import Path
    import pandas as pd
    import casadi as ca

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

    x_ini, u_ini = busca_condicao_inicial(data_hora, estimador_vazao, df)

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

    entradas = u_ini + x_ini

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

    HP = 3
    HC = HP-1

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

    # Tamanho da matriz que vai indicar as variáveis controladas por setpoint
    matriz_h = np.zeros((2, nx))

    # PChegada - Coluna na linha 1 que indica a primeira variável controlada
    matriz_h[0, 1] = 1

    # Vazao - Coluna na linha 2 que indica a segunda variável controlada
    matriz_h[1, 10] = 1

    # Criação da variável simbólica para os estados medidos
    estados_medidos_sym = ca.MX.sym('estados_medidos', nx, 1)

    # Função de saída que mapeia diretamente o estado para a saída
    funcao_h = ca.Function('h',
                           [estados_medidos_sym],
                           [ca.mtimes(matriz_h, estados_medidos_sym)])

    wall_time = 10  # Tempo máximo de execução

    # Tabela completa com pré-cálculos dos limites dinâmicos em função da frequência
    arquivo_limites_integrados = 'TabelaLimitesDinamicos.xlsx'
    caminho_tabelas = Path("./Tabelas")
    df_limites_integrados = pd.read_excel(
        caminho_tabelas / arquivo_limites_integrados)
    matriz_limites_integrados = df_limites_integrados.values

    busca_limites = cria_busca_limites_casadi(matriz_limites_integrados)

    solver = cria_solver(umax, umin, dumax, margem_percentual,
                         HP, HC, matriz_qy, matriz_qu, matriz_r, matriz_qx, nx, nu, ny,
                         estimador_vazao, busca_limites, modelo_preditor, funcao_h, wall_time)
