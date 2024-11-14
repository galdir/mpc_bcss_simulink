import casadi
import numpy as np

def cria_solver(umax, umin, dumax, MargemPercentual, 
                Hp, Hc, Qy, Qu, R, Qx, nx, nu, ny, 
                EstimaVazao, buscaLimites, ModeloPreditor, funcao_h, WallTime):
    """
    Cria um solver MPC usando CasADi.
    
    Mantém os mesmos parâmetros da versão MATLAB, preservando a funcionalidade
    e nomenclatura em português.
    """
    # Inicialização do timer e importação do CasADi
    nx_ESN = len(ModeloPreditor.data.a0)
    
    # Variáveis simbólicas para o problema de otimização
    X = casadi.MX.sym('X', nx, 1+Hp)  # Estado atual + Estados futuros até Hp
    U = casadi.MX.sym('U', nu, Hp)    # Ações de controle até o horizonte Hp
    DU = casadi.MX.sym('DU', nu, Hp)  # Variações nas ações de controle sobre o horizonte Hp
    
    # Parâmetros que foram oferecidos para o Solver
    Indice = [
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
    P = casadi.MX.sym('P', sum(Indice))
    
    # Associa variáveis simbólicas as respectivas partes no vetor de parâmetros
    Xk0, Uk0, AlvoEng, Ysp, ErroX, ErroY, BuffDeltaFreq, Reservatorio_ESN = ExtraiParametros(P, Indice)
    
    ModeloPreditor_estado_atual = Reservatorio_ESN
    
    # Inicializa variável que vai armazenar a estrutura de argumentos do NLP
    args = {
        'lbx': [],  # Limites inferiores para as restrições dos estados do MPC
        'ubx': [],  # Limites superiores para as restrições dos estados do MPC
    }
    
    # Para os estados atuais e para todo o horizonte Hp
    for k in range(1+Hp):
        args['lbx'].extend([0] * nx)
        args['ubx'].extend([float('inf')] * nx)
    
    # Para as ações de controle em todo o horizonte futuro
    for k in range(Hp):
        args['lbx'].append(umin)
        args['ubx'].append(umax)
        
    # Restrições para os limites na variação das ações de controle
    for k in range(Hp):
        args['lbx'].append(-dumax)
        args['ubx'].append(dumax)
    
    # Montando as restrições de igualdade/desigualdade em g
    g = []
    args['lbg'] = []
    args['ubg'] = []
    
    # Multipleshooting = restrições de igualdade para seguir dinâmica do sistema
    g.append(X[:, 0] - Xk0)
    args['lbg'].extend([0] * nx)
    args['ubg'].extend([0] * nx)
    
    # Limita a primeira ação de controle pelo máximo DeltaU
    g.append(U[:, 0] - Uk0 - DU[:, 0])
    args['lbg'].extend([0] * nu)
    args['ubg'].extend([0] * nu)
    
    # Restrições de igualdade para assegurar que os estados futuros vão seguir as predições
    for k in range(Hp):
        x_predito, ModeloPreditor_novo_estado = executa_predicao(
            U[:, k], X[:, k], ModeloPreditor_estado_atual, ModeloPreditor, EstimaVazao)
        ModeloPreditor_estado_atual = ModeloPreditor_novo_estado
        
        g.append(X[:, k+1] - x_predito)
        args['lbg'].extend([0] * nx)
        args['ubg'].extend([0] * nx)
    
    # Restrições de igualdade para definir DeltaU em função de U
    for k in range(Hp-1):
        Soma = U[:, k+1] - U[:, k] - DU[:, k+1]
        g.append(Soma)
        args['lbg'].extend([0] * nu)
        args['ubg'].extend([0] * nu)
        
        # Restrição avaliando as variações acumuladas na frequencia
        BuffDeltaFreq = casadi.vertcat(DU[0, k], BuffDeltaFreq[:-1])
        Soma = casadi.sum1(BuffDeltaFreq)
        
        g.append(Soma)
        args['lbg'].append(-1)
        args['ubg'].append(1)
    
    # Restrições dinâmicas para os estados X e para as saidas Y
    for k in range(Hp):
        LimitesX = buscaLimites(U[0, k]).T
        LimitesX[:, 0] *= (1 - MargemPercentual/100)
        LimitesX[:, 1] *= (1 + MargemPercentual/100)
        
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
    for k in range(Hc, Hp):
        g.append(U[:, k] - U[:, k-1])
        args['lbg'].extend([0] * nu)
        args['ubg'].extend([0] * nu)
    
    # Preparando o custo da função objetivo
    fob = 0
    fob += casadi.mtimes(casadi.mtimes(ErroX.T, Qx), ErroX)
    
    for k in range(Hp):
        y_saida = funcao_h(X[:, k+1])
        erro_termo = y_saida - Ysp + ErroY
        fob += casadi.mtimes(casadi.mtimes(erro_termo.T, Qy), erro_termo)
    
    for k in range(Hc):
        erro_controle = U[:, k] - AlvoEng
        fob += casadi.mtimes(casadi.mtimes(erro_controle.T, Qu), erro_controle)
        fob += casadi.mtimes(casadi.mtimes(DU[:, k].T, R), DU[:, k])
    
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
        'p': P
    }
    
    # Configuração específica do IPOPT
    opcoes = {
        'print_time': 0,
        'ipopt': {
            'print_level': 0,
            'bound_relax_factor': 0,
            'max_iter': 1000,
            'max_wall_time': WallTime
        },
        'verbose': 0
    }
    
    # Define o solver IPOPT para resolver o problema de otimização não linear
    solver = casadi.nlpsol('solver', 'ipopt', nlp, opcoes)
    
    return solver, args

def ExtraiParametros(P, Indice):
    """
    Extrai as partes que compõe os parâmetros do Solver.
    
    Similar à função ExtraiParametros do MATLAB.
    """
    soma_acumulada = np.cumsum([0] + Indice)
    fatias = [slice(inicio, fim) for inicio, fim in zip(soma_acumulada[:-1], soma_acumulada[1:])]
    
    return [P[s] for s in fatias]