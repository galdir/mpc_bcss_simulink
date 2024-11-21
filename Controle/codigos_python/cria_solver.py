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
    # g.append(U[:, 0] - uk0 - DU[:, 0])
    g = ca.vertcat(g, u_sym[:, 0] - uk0 - du_sym[:, 0])
    args['lbg'].extend([0] * nu)
    args['ubg'].extend([0] * nu)

    # Restrições de igualdade para assegurar que os estados futuros vão seguir as predições
    for k in range(hp):
        x_predito, modelo_preditor_novo_estado = executa_predicao_casadi(
            ca.transpose(u_sym[:, k]), ca.transpose(x_sym[:, k]), modelo_preditor_estado_atual, modelo_preditor, estima_vazao)

        modelo_preditor_estado_atual = modelo_preditor_novo_estado

        # testando com preditor ingenuo
        # x_predito = (u_sym[1,k]/uk0[1]) * x_sym[:, k]

        # g.append(X[:, k+1] - x_predito)
        g = ca.vertcat(g, x_sym[:, k+1] - x_predito)
        args['lbg'].extend([0] * nx)
        args['ubg'].extend([0] * nx)

    # Restrições de igualdade para definir DeltaU em função de U
    for k in range(hp-1):
        # Cálculo da variação na ação de controle = DeltaU
        soma = u_sym[:, k+1] - u_sym[:, k] - du_sym[:, k+1]
        # g.append(Soma)
        g = ca.vertcat(g, soma)
        args['lbg'].extend([0] * nu)
        args['ubg'].extend([0] * nu)

        # Restrição avaliando as variações acumuladas na frequencia
        buff_delta_freq = ca.vertcat(du_sym[0, k], buff_delta_freq[:-1])
        soma = ca.sum1(buff_delta_freq)

        # g.append(Soma)
        g = ca.vertcat(g, soma)
        args['lbg'].append(-1)
        args['ubg'].append(1)

    # Restrições dinâmicas para os estados X e para as saidas Y
    for k in range(hp):
        limites_x = busca_limites(u_sym[0, k]).T
        limites_x[:, 0] *= (1 - margem_percentual/100)
        limites_x[:, 1] *= (1 + margem_percentual/100)

        limites_y = funcao_h(limites_x)

        # Restrições para as variáveis do processo (estados X)
        limites_x_max = limites_x[:, 0] - x_sym[:, k+1]
        # g.extend(limites_x_max)
        g = ca.vertcat(g, limites_x_max)
        args['lbg'].extend([0] * nx)
        args['ubg'].extend([float('inf')] * nx)

        limites_x_min = x_sym[:, k+1] - limites_x[:, 1]
        # g.extend(limites_x_min)
        g = ca.vertcat(g, limites_x_min)
        args['lbg'].extend([0] * nx)
        args['ubg'].extend([float('inf')] * nx)

        # Restrições para as variáveis de saida
        y_saida = funcao_h(x_sym[:, k+1])

        limites_y_max = limites_y[:, 0] - y_saida
        # g.extend(limites_y_max)
        g = ca.vertcat(g, limites_y_max)
        args['lbg'].extend([0] * ny)
        args['ubg'].extend([float('inf')] * ny)

        limites_y_min = y_saida - limites_y[:, 1]
        # g.extend(limites_y_min)
        g = ca.vertcat(g, limites_y_min)
        args['lbg'].extend([0] * ny)
        args['ubg'].extend([float('inf')] * ny)

        # Limites PMonAlvo
        limite_min_variavel_alvo = ca.fmax(limites_x[1, 1], umin[1])
        diferenca_min = u_sym[1, k] - limite_min_variavel_alvo
        # g.append(diferenca_min)
        g = ca.vertcat(g, diferenca_min)
        args['lbg'].append(0)
        args['ubg'].append(float('inf'))

        limite_max_variavel_alvo = ca.fmin(limites_x[1, 0], umax[1])
        diferenca_max = limite_max_variavel_alvo - u_sym[1, k]
        # g.append(diferenca_max)
        g = ca.vertcat(g, diferenca_max)
        args['lbg'].append(0)
        args['ubg'].append(float('inf'))

    # Depois do instante Hc, seguir a teoria e manter a mesma ação de controle futura
    for k in range(hc, hp):
        # g.extend(U[:, k] - U[:, k-1])
        g = ca.vertcat(g, u_sym[:, k] - u_sym[:, k-1])
        args['lbg'].extend([0] * nu)
        args['ubg'].extend([0] * nu)

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
        # 'g': ca.vertcat(*g),
        'g': g,
        'p': parametros_sym
    }

    # Configuração específica do IPOPT
    opcoes = {
        'print_time': 0,
        'ipopt': {
            'print_level': 0,
            'bound_relax_factor': 0,
            'max_iter': 1000,
            'max_wall_time': walltime,
            'print_user_options': 'yes',
            'print_timing_statistics': 'yes',
            # 'print_advanced_options': 'yes',
            # 'print_options_documentation': 'yes',
            # 'sb': 'no',
            # 'regularization': 'no_regularization',
            # 'constr_viol_tol': 0.5,
            # 'acceptable_tol': 0.5

        },
        'verbose': 0,
        # 'expand': True,
        # 'jit': True,
        # 'regularity_check': True,
        # 'print_in': True,
        # 'inputs_check': True,
        # 'verbose_init': True,
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
