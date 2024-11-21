import casadi as ca
import numpy as np

from limites import cria_busca_limites_casadi
from estima_vazao import cria_estimador_vazao_casadi
from predicao import esquentar_esn, executa_predicao_casadi


def cria_solver(umax, umin, dumax, margem_percentual,
                hp, hc, matriz_qy, matriz_qu, matriz_r, matriz_qx, nx, nu, ny,
                estima_vazao, busca_limites, modelo_preditor, funcao_h, walltime):
    """
    Cria um solver MPC usando ca.

    Mantém os mesmos parâmetros da versão MATLAB, preservando a funcionalidade
    e nomenclatura em português.
    """

    dumin = [-x for x in dumax]

    nx_ESN = len(modelo_preditor['data']['a0'][0][0])

    # Variáveis simbólicas para o problema de otimização
    x_sym = ca.MX.sym('X', nx, 1+hp)  # Estado atual + Estados futuros até Hp
    u_sym = ca.MX.sym('U', nu, hp)    # Ações de controle até o horizonte Hp
    # Variações nas ações de controle sobre o horizonte Hp
    du_sym = ca.MX.sym('DU', nu, hp)

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
    parametros_sym = ca.MX.sym('p', sum(indice_parametros))

    # Associa variáveis simbólicas as respectivas partes no vetor de parâmetros
    xk0, uk0, alvo_eng, ysp, erro_x, erro_y, buff_delta_freq, modelo_preditor_estado_atual = extrai_parametros(
        parametros_sym, indice_parametros)

    # modelo_preditor_estado_atual = reservatorio_esn

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
        args['lbx'].extend(umin)
        args['ubx'].extend(umax)

    # Restrições para os limites na variação das ações de controle
    for k in range(hp):
        args['lbx'].extend(dumin)
        args['ubx'].extend(dumax)

    # Montando as restrições de igualdade/desigualdade em g
    g = []
    args['lbg'] = []
    args['ubg'] = []

    # Multipleshooting = restrições de igualdade para seguir dinâmica do sistema
    g = ca.vertcat(g, x_sym[:, 0] - xk0)
    args['lbg'].extend([0] * nx)
    args['ubg'].extend([0] * nx)

    # Limita a primeira ação de controle pelo máximo DeltaU
    #g.append(U[:, 0] - uk0 - DU[:, 0])
    g = ca.vertcat(g, u_sym[:, 0] - uk0 - du_sym[:, 0])
    args['lbg'].extend([0] * nu)
    args['ubg'].extend([0] * nu)

    # Restrições de igualdade para assegurar que os estados futuros vão seguir as predições
    for k in range(hp):
        # x_predito, modelo_preditor_novo_estado = executa_predicao_casadi(
        #     ca.transpose(U[:, k]), ca.transpose(X[:, k]), modelo_preditor_estado_atual, modelo_preditor, estima_vazao)
        
        # modelo_preditor_estado_atual = modelo_preditor_novo_estado
        
        #testando com preditor ingenuo
        x_predito = (u_sym[1,k]/uk0[1]) * x_sym[:, k]
        
        #g.append(X[:, k+1] - x_predito)
        g = ca.vertcat(g, x_sym[:, k+1] - x_predito)
        args['lbg'].extend([0] * nx)
        args['ubg'].extend([0] * nx)

    # Restrições de igualdade para definir DeltaU em função de U
    for k in range(hp-1):
        #Cálculo da variação na ação de controle = DeltaU
        soma = u_sym[:, k+1] - u_sym[:, k] - du_sym[:, k+1]
        #g.append(Soma)
        g = ca.vertcat(g, soma)
        args['lbg'].extend([0] * nu)
        args['ubg'].extend([0] * nu)

        # Restrição avaliando as variações acumuladas na frequencia
        #buff_delta_freq = ca.vertcat(du_sym[0, k], buff_delta_freq[:-1])
        #soma = ca.sum1(buff_delta_freq)

        #g.append(Soma)
        #g = ca.vertcat(g, soma)
        #args['lbg'].append(-1)
        #args['ubg'].append(1)

    # Restrições dinâmicas para os estados X e para as saidas Y
    # for k in range(hp):
    #     limites_x = busca_limites(u_sym[0, k]).T
    #     limites_x[:, 0] *= (1 - margem_percentual/100)
    #     limites_x[:, 1] *= (1 + margem_percentual/100)

    #     limites_y = funcao_h(limites_x)

    #     # Restrições para as variáveis do processo (estados X)
    #     limites_x_max = limites_x[:, 0] - x_sym[:, k+1]
    #     #g.extend(limites_x_max)
    #     g = ca.vertcat(g, limites_x_max)
    #     args['lbg'].extend([0] * nx)
    #     args['ubg'].extend([float('inf')] * nx)

    #     limites_x_min = x_sym[:, k+1] - limites_x[:, 1]
    #     #g.extend(limites_x_min)
    #     g = ca.vertcat(g, limites_x_min)
    #     args['lbg'].extend([0] * nx)
    #     args['ubg'].extend([float('inf')] * nx)

    #     # Restrições para as variáveis de saida
    #     y_saida = funcao_h(x_sym[:, k+1])

    #     limites_y_max = limites_y[:, 0] - y_saida
    #     #g.extend(limites_y_max)
    #     g = ca.vertcat(g, limites_y_max)
    #     args['lbg'].extend([0] * ny)
    #     args['ubg'].extend([float('inf')] * ny)

    #     limites_y_min = y_saida - limites_y[:, 1]
    #     #g.extend(limites_y_min)
    #     g = ca.vertcat(g, limites_y_min)
    #     args['lbg'].extend([0] * ny)
    #     args['ubg'].extend([float('inf')] * ny)

    #     # Limites PMonAlvo
    #     limite_min_variavel_alvo = ca.fmax(limites_x[1, 1], umin[1])
    #     diferenca_min = u_sym[1, k] - limite_min_variavel_alvo
    #     #g.append(diferenca_min)
    #     g = ca.vertcat(g, diferenca_min)
    #     args['lbg'].append(0)
    #     args['ubg'].append(float('inf'))

    #     limite_max_variavel_alvo = ca.fmin(limites_x[1, 0], umax[1])
    #     diferenca_max = limite_max_variavel_alvo - u_sym[1, k]
    #     #g.append(diferenca_max)
    #     g = ca.vertcat(g, diferenca_max)
    #     args['lbg'].append(0)
    #     args['ubg'].append(float('inf'))

    # # Depois do instante Hc, seguir a teoria e manter a mesma ação de controle futura
    # for k in range(hc, hp):
    #     #g.extend(U[:, k] - U[:, k-1])
    #     g = ca.vertcat(g, u_sym[:, k] - u_sym[:, k-1])
    #     args['lbg'].extend([0] * nu)
    #     args['ubg'].extend([0] * nu)

    # Preparando o custo da função objetivo
    fob = 0
    fob += ca.mtimes(ca.mtimes(erro_x.T, matriz_qx), erro_x)

    for k in range(hp):
        y_saida = funcao_h(x_sym[:, k+1])
        erro_termo = y_saida - ysp + erro_y
        fob += ca.mtimes(ca.mtimes(erro_termo.T, matriz_qy), erro_termo)

    for k in range(hc):
        erro_controle = u_sym[:, k] - alvo_eng
        fob += ca.mtimes(ca.mtimes(erro_controle.T, matriz_qu), erro_controle)
        fob += ca.mtimes(ca.mtimes(du_sym[:, k].T, matriz_r), du_sym[:, k])

    # Monta as variáveis de decisão em um vetor coluna
    variaveis_opt = ca.vertcat(
        ca.vec(x_sym),
        ca.vec(u_sym),
        ca.vec(du_sym)
    )

    # Define a estrutura para problema de otimização não linear
    nlp = {
        'f': fob,
        'x': variaveis_opt,
        #'g': ca.vertcat(*g),
        'g': g,
        'p': parametros_sym
    }

    # Configuração específica do IPOPT
    opcoes = {
        'print_time': 1,
        'ipopt': {
            'print_level': 12,
            'bound_relax_factor': 0,
            'max_iter': 1000,
            'max_wall_time': walltime,
            'print_user_options': 'yes',
            'print_timing_statistics': 'yes',
            #'print_advanced_options': 'yes',
            # 'print_options_documentation': 'yes',
            #'sb': 'no',
            #'regularization': 'no_regularization',
            #'constr_viol_tol': 0.5,
            #'acceptable_tol': 0.5
            
        },
        'verbose': 1,
        #'expand': True,
        #'jit': True,
        #'regularity_check': True,
        #'print_in': True,
        #'inputs_check': True,
        #'verbose_init': True,
    }

    # Define o solver IPOPT para resolver o problema de otimização não linear
    solver = ca.nlpsol('solver', 'ipopt', nlp, opcoes)

    return solver, args


def extrai_parametros(parametros, indice_parametros):
    """
    Extrai as partes que compõe os parâmetros do Solver.

    Similar à função ExtraiParametros do MATLAB.
    """
    soma_acumulada = np.cumsum([0] + indice_parametros)
    fatias = [slice(inicio, fim) for inicio, fim in zip(
        soma_acumulada[:-1], soma_acumulada[1:])]

    return [parametros[s] for s in fatias]


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


def debug_solver_setup(solver, args, x0, p):
    """
    Verifica a configuração inicial do solver com suporte adequado para objetos CasADi
    """
    print("\nDEBUG SOLVER SETUP")
    print("-----------------")

    # 1. Verificar dimensões
    print("\n1. Dimensões:")
    print(f"Número de variáveis (x): {len(args['lbx'])}")
    print(f"Número de restrições (g): {len(args['lbg'])}")
    print(f"Tamanho do x0: {x0.size1() if isinstance(x0, ca.MX) else len(x0)}")
    print(
        f"Tamanho dos parâmetros p: {p.size1() if isinstance(p, ca.MX) else len(p)}")

    # 2. Verificar limites
    print("\n2. Verificação de limites:")
    print(f"lbx length: {len(args['lbx'])}")
    print(f"ubx length: {len(args['ubx'])}")
    print(f"lbg length: {len(args['lbg'])}")
    print(f"ubg length: {len(args['ubg'])}")

    # 3. Verificar valores iniciais
    print("\n3. Verificação de valores iniciais:")
    if isinstance(x0, (np.ndarray, list)):
        x0_array = np.array(x0)
    else:
        # Se for um objeto CasADi, vamos usar os valores originais de solucao_anterior
        x0_array = np.array(solucao_anterior)

    x0_inf = np.isinf(x0_array)
    x0_nan = np.isnan(x0_array)
    if np.any(x0_inf):
        print("ERRO: x0 contém valores infinitos nas posições:",
              np.where(x0_inf)[0])
    if np.any(x0_nan):
        print("ERRO: x0 contém NaN nas posições:", np.where(x0_nan)[0])

    # 4. Verificar parâmetros
    print("\n4. Verificação de parâmetros:")
    if isinstance(p, (np.ndarray, list)):
        p_array = np.array(p)
    else:
        # Se for um objeto CasADi, vamos usar os valores originais de args['p']
        p_array = np.array(args['p'])

    p_inf = np.isinf(p_array)
    p_nan = np.isnan(p_array)
    if np.any(p_inf):
        print("ERRO: p contém valores infinitos nas posições:",
              np.where(p_inf)[0])
    if np.any(p_nan):
        print("ERRO: p contém NaN nas posições:", np.where(p_nan)[0])

    # 5. Verificar limites consistentes
    print("\n5. Verificação de consistência de limites:")
    for i in range(len(args['lbx'])):
        if args['lbx'][i] > args['ubx'][i]:
            print(
                f"ERRO: Limite inconsistente na variável {i}: lb={args['lbx'][i]} > ub={args['ubx'][i]}")

    for i in range(len(args['lbg'])):
        if args['lbg'][i] > args['ubg'][i]:
            print(
                f"ERRO: Limite inconsistente na restrição {i}: lb={args['lbg'][i]} > ub={args['ubg'][i]}")

    # 6. Verificar x0 dentro dos limites
    print("\n6. Verificação de x0 dentro dos limites:")
    for i in range(len(x0_array)):
        if i < len(args['lbx']) and (x0_array[i] < args['lbx'][i] or x0_array[i] > args['ubx'][i]):
            print(
                f"ERRO: x0[{i}]={x0_array[i]} está fora dos limites [{args['lbx'][i]}, {args['ubx'][i]}]")

    # 7. Mostrar valores das primeiras restrições
    print("\n7. Amostra de valores iniciais e limites:")
    n_sample = min(5, len(x0_array))
    print("\nPrimeiros valores de x0:")
    for i in range(n_sample):
        print(
            f"x0[{i}] = {x0_array[i]}, limites: [{args['lbx'][i]}, {args['ubx'][i]}]")

    print("\n8. Verificação de estrutura do problema:")
    print(f"Número de variáveis de estado por passo: {nx}")
    print(f"Número de variáveis de controle por passo: {nu}")
    print(f"Horizonte de predição (hp): {hp}")
    print(f"Horizonte de controle (hc): {hc}")

    # Adicionar informações sobre as dimensões esperadas
    # estados + controles + delta controles
    n_vars_expected = nx * (hp + 1) + nu * hp + nu * hp
    print(f"\nDimensões esperadas:")
    print(f"Número total de variáveis esperado: {n_vars_expected}")
    print(f"Número atual de variáveis: {len(args['lbx'])}")
    if n_vars_expected != len(args['lbx']):
        print("ERRO: Dimensão das variáveis não corresponde ao esperado!")

    return


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

    solver, args = cria_solver(umax, umin, dumax, margem_percentual,
                               hp, hc, matriz_qy, matriz_qu, matriz_r, matriz_qx, nx, nu, ny,
                               estimador_vazao, busca_limites, modelo_preditor, funcao_h, wall_time)

    type(solver)

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
    # args={}
    args['p'] = np.hstack([x_atual, u_atual, alvos_eng, ysp, erro_x,
                          erro_y, buff_delta_freq, reservatorio_esn])

    # Uso:
    # try:
    #     print("Iniciando verificação do setup do solver...")
    #     debug_solver_setup(
    #         solver,
    #         args,
    #         ca.MX(solucao_anterior),  # x0
    #         ca.MX(args['p'])  # p
    #     )

    #     print("\nTentando executar o solver...")
    #     solucao = solver(
    #         x0=ca.MX(solucao_anterior),
    #         lbx=ca.MX(args['lbx']),
    #         ubx=ca.MX(args['ubx']),
    #         lbg=ca.MX(args['lbg']),
    #         ubg=ca.MX(args['ubg']),
    #         p=ca.MX(args['p'])
    #     )
    #     print("Solver executado com sucesso")

    # except Exception as e:
    #     print(f"\nERRO durante a execução: {str(e)}")
    #     if hasattr(e, '__traceback__'):
    #         import traceback
    #         print("\nTraceback completo:")
    #         traceback.print_tb(e.__traceback__)

    solucao = solver(
        x0=ca.MX(solucao_anterior),
        lbx=ca.MX(args['lbx']),
        ubx=ca.MX(args['ubx']),
        lbg=ca.MX(args['lbg']),
        ubg=ca.MX(args['ubg']),
        p=ca.MX(args['p'])
    )

    feasible = solver.stats()['success']
    iteracoes = solver.stats()['iter_count']

    print('feasible:')
    print(feasible)
    print('iteracoes:')
    print(iteracoes)
    # print(f"Constraint violations: {solver.stats()['g_violation']}")
