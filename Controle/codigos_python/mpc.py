import casadi as cs
import numpy as np
import pandas as pd
import scipy.io as sio
import os

from forecasting import get_ESN_models_and_required_variables
from interpola_casadi_vazao import interpola_casadi_vazao
from normalizacao import desnormaliza_predicoes, normaliza_entradas

def inicializa_solver(Hp, Hc, Qy, Qu, R, ny, nu, nx, ModeloPreditor, matrizVazao):
    """
    Parâmetros desta função:
    Hp = Horizonte de predição
    Hc = Horizonte de controle
    Qy = Matriz diagonal para ponderação das variáveis controladas
    Qu = Matriz diagonal para ponderação dos alvos desejados
    R = Matriz diagonal para ponderação das variáveis manipuladas ao longo de todo o horizonte de controle
    ny = Número de variáveis controladas (no caso, 2: PSuc e PChegada)
    nu = Número de variáveis manipuladas (no caso, 2: Freq e PMonAlvo)
    nx = Número de variáveis coletadas do processo (no caso, 10)
    ModeloPreditor = rede utilizada como preditor interno ao controlador
    """

    # Define parâmetros simbólicos (Casadi) gerais para o problema de otimização
    X = cs.MX.sym('X', nx, Hp + 1)  # Predição dos estados sobre o horizonte Hp
    du = cs.MX.sym('du', Hc * nu, 1)  # Incrementos do controle sobre o horizonte Hc (Variável de decisão)
    Du = cs.vertcat(du, cs.MX.zeros((nu * (Hp - Hc), 1)))  # Sequência de ação de controle
    ysp = cs.MX.sym('ysp', ny, 1)  # Set-point ótimo calculado pelo otimizador (Variável de decisão)
    VarControladas = cs.MX.sym('X_MPC', nx)  # Cria uma função de saída (h) em função dos estados (Psuc e Pcheg) para controladas
    h = cs.Function('h', [VarControladas], [VarControladas[:2]])  # Define os dois primeiros estados como controladas (Psuc e Pcheg) para o solver comparar com setpoint (ysp)

    print('Usando uma estrutura ESN como preditor para o MPC')
    sol_args = {}  # Inicializa dicionário que vai armazenar a estrutura de argumentos

    # Parâmetros simbólicos específicos da ESN
    nx_ESN = len(ModeloPreditor['data']['Wir'][0][0])  # Resgata o tamanho do reservatório da ESN utilizada como modelo preditor

    # P = quantidade de parâmetros para o Solver. Os P parâmetros são:
    # - DadosProcesso (dimensão=nx)
    # - uk(entradas) (dimensão=nu)
    # - Erro, sendo a diferença entre a medição do processo e a última predição das variáveis controladas (dimensão=ny)
    # - Alvo dado pelo RTO (dimensão=nu)
    # - Dados do reservatório da ESN utilizada pelo controlador (dimensão=nx_ESN)
    P = cs.MX.sym('P', nx + nu + ny + nu + nx_ESN)  # qtd de parâmetros para o Solver
    uk_1 = P[nx:nx+nu]  # define variável simbólica das entradas (Freq. PmonAlvo)
    erro = P[nx+nu:nx+nu+ny]  # define variável simbólica para erro (DadosProcesso-PrediçãoMPC) ->(Psuc. Pcheg)
    uRTO = P[nx+nu+ny:nx+nu+ny+nu]  # define variável simbólica para Alvo (Freq. e PmonAlvo)
    ESNdataa0 = P[nx+nu+ny+nu:]  # define variável simbólica do reservatório da ESN
    u = cs.vertcat(uk_1, P[:nx])
    g = [X[:, 0] - P[:nx]]  # define variável que vai empilhar as restrições durante o Hp

    # Montando funções com expressões casadi para acelerar o loop de horizonte de predição
    Press_sym = cs.MX.sym('Press_sym')
    Freq_sym = cs.MX.sym('Freq_sym')

    Interpola_casadi_vazao_sym = interpola_casadi_vazao(Freq_sym, Press_sym, matrizVazao)
    f_Interpola_casadi_vazao_sym = cs.Function('f_vazao', [Freq_sym, Press_sym], [Interpola_casadi_vazao_sym])

    # Define a função objetivo (fob) de forma recursiva ao longo de Hp passos, utilizando o modelo preditor para otimizar as variáveis de controle, considerando as restrições do processo.
    fob = 0
    for k in range(Hp):
        uk_1 = uk_1 + Du[k*nu:(k+1)*nu]  # define variável simbólica para soma dos incrementos de controle
        ym = h(X[:, k+1])  # define variável simbólica que será controlada utilizando a função de saída (h) definida anteriormente
        fob += (ym - ysp + erro).T @ Qy @ (ym - ysp + erro) + du.T @ R @ du + (uk_1 - uRTO).T @ Qu @ (uk_1 - uRTO)  # define a função objetivo proposta
        u = cs.vertcat(uk_1, P[:nx])  # define uma matriz para armazenar as variáveis de entrada no modeloPreditor
        
        y_esn_pred, ESNdataa0 = executa_predicao_ESN_vazao_casadi(u, ESNdataa0, ModeloPreditor, f_Interpola_casadi_vazao_sym)
        
        g.append(X[:, k+1] - y_esn_pred)  # Define variável simbólica para atender as restrições nos LimitesInferior e LimiteSuperior(lbg<g(x)<ubg)

    # Define as matrizes auxiliares (Mtil, Itil) para o incremento de controle (ação de controle) ao longo de Hc passos, no conjunto de restrições (g)
    Mtil = []
    Itil = []
    auxM = np.zeros((nu, Hc*nu))
    for i in range(Hc):
        auxM = np.hstack((np.eye(nu), auxM[:, :(Hc-1)*nu]))
        Mtil.append(auxM)
        Itil.append(np.eye(nu))
    Mtil = np.vstack(Mtil)
    Itil = np.vstack(Itil)

    # Conclui a inclusão das restrições nos estados e nas entradas.
    g.append(Mtil @ du + Itil @ P[nx:nx+nu])

    # Configuração do otimizador
    opt_variable = cs.vertcat(cs.reshape(X, -1, 1), du, ysp)  # variáveis calculadas pelo Solver(predição;incrementos de controle;set-point*)
    nlp = {'f': fob, 'x': opt_variable, 'g': cs.vertcat(*g), 'p': P}  # define a estrutura para problema de otimização não linear (NLP, Nonlinear Programming)

    # Configuração específica do otimizador
    options = {
        'print_time': 0,  # Habilita tempo total de execução do solver deve ser impresso ou não.
        'ipopt': {
            'print_level': 1,  # Nível de detalhamento das mensagens de saída do IPOPT. Valores mais baixos resultam em menos mensagens (0 significa sem mensagens).
            'max_iter': 100,  # Especifica o número máximo de iterações que o solver deve executar antes de parar.
            'acceptable_tol': 1e-4,  # Define a tolerância de convergência do solver. Um valor menor indica uma solução mais precisa.
            'acceptable_obj_change_tol': 1e-4  # Define uma tolerância aceitável para uma solução "boa o suficiente", útil para problemas onde a solução perfeita pode ser muito difícil de alcançar.
        }
    }
    SolucaoOtimizador = cs.nlpsol('SolucaoOtimizador', 'ipopt', nlp, options)  # Define o Interior Point OPTimizer (ipopt) para resolver o problema de otimização não linear (nlp)
    sol_args['solucionador'] = SolucaoOtimizador

    return sol_args

def executa_predicao_ESN_vazao_casadi(entradas, ESNdataa0, modelo_ESN, f_matrizVazao_sym):
    """
    entradas é um vetor coluna: frequencia_BCSS, pressao_montante_alvo, ...
              pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
              temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, ...
              temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
      
    modelo_ESN precisa ter: ['data']['Wrr'], ['data']['Wir'], ['data']['Wbr']
    
    ESNdataa0 é o estado do reservatório da esn após a última predição
    
    matrizVazao é a matriz com vazões estimadas por frequência e pressão de chegada
    
    saídas é um vetor coluna com o instante seguinte para frequencia_BCSS, pressao_montante_alvo:
              pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
              temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, ...
              temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada,
              vazaoOleo
    """
    predicoes, novo_a0 = executa_predicao_ESN_casadi(entradas, ESNdataa0, modelo_ESN)
    
    vazaoOleo_estimada = f_matrizVazao_sym(entradas[0], predicoes[1] * 1.019716)
    
    predicoes = cs.vertcat(predicoes, vazaoOleo_estimada)
    
    return predicoes, novo_a0

def executa_predicao_ESN_casadi(entradas, ESNdataa0, modelo_ESN):
    """
    entradas é um vetor coluna: frequencia_BCSS, pressao_montante_alvo, ...
              pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
              temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, ...
              temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
      
    modelo_ESN precisa ter: ['data']['Wrr'], ['data']['Wir'], ['data']['Wbr']
    
    ESNdataa0 é o estado do reservatório da esn após a última predição
    
    saídas é um vetor coluna com o instante seguinte para frequencia_BCSS, pressao_montante_alvo:
              pressao_succao_BCSS, pressao_chegada, pressao_diferencial_BCSS, pressao_descarga_BCSS, ...
              temperatura_motor_BCSS, corrente_torque_BCSS, corrente_total_BCSS, ...
              temperatura_succao_BCSS, vibracao_BCSS, temperatura_chegada
    """
    entradas_normalizadas = normaliza_entradas(entradas)
    # wrr = modelo_ESN['data']['Wrr'][0][0]
    # wir = modelo_ESN['data']['Wir'][0][0]
    # wbr = modelo_ESN['data']['Wbr'][0][0]
    # wro = modelo_ESN['data']['Wro'][0][0]
    # gama = modelo_ESN['data']['gama'][0][0][0][0]
    wrr = cs.MX(modelo_ESN['data']['Wrr'][0][0])
    wir = cs.MX(modelo_ESN['data']['Wir'][0][0])
    wbr = cs.MX(modelo_ESN['data']['Wbr'][0][0])
    wro = cs.MX(modelo_ESN['data']['Wro'][0][0])
    gama = cs.MX(modelo_ESN['data']['gama'][0][0][0][0])


    if isinstance(ESNdataa0, np.ndarray):
        ESNdataa0 = cs.MX(ESNdataa0)
    if isinstance(entradas_normalizadas, np.ndarray):
        entradas_normalizadas = cs.MX(entradas_normalizadas)

    x_ESN = cs.mtimes(wrr, ESNdataa0) + cs.mtimes(wir, entradas_normalizadas) + wbr
    #x_ESN = wrr @ ESNdataa0 + wir @ entradas_normalizadas + wbr  # usar o modeloPreditor(ESN) para fazer a predição
    novo_a0 = (1 - gama) * ESNdataa0 + gama * cs.tanh(x_ESN)  # Atualiza estado da ESN
    a_wbias = cs.vertcat(1.0, novo_a0)
    predicoes_normalizadas = wro * a_wbias

    predicoes = desnormaliza_predicoes(predicoes_normalizadas)

    return predicoes, novo_a0

# Nota: As funções normaliza_entradas, desnormaliza_predicoes e Interpola_casadi_vazao 
# não foram fornecidas no código original, então elas precisarão ser implementadas separadamente.


def teste_InicializaSolver_limpo():
    MODELS_FOLDER = './Modelos/ESN/'
    df_simulador = pd.read_excel('./Tabelas/DoSimulador.xlsx')      
    matriz_simulador_vazao = df_simulador.iloc[1:,:3].values

    # Defina os parâmetros de entrada
    Hp = 10  # Horizonte de predição
    Hc = 5   # Horizonte de controle
    ny = 2   # Número de variáveis controladas
    nu = 2   # Número de variáveis manipuladas
    nx = 10  # Número de variáveis coletadas do processo

    # Crie matrizes de ponderação
    Qy = np.eye(ny)
    Qu = np.eye(nu)
    R = np.eye(nu * Hc)

    # Crie um modelo preditor fictício (você precisa adaptar isso para o seu caso específico)
    # class ModeloPreditorFicticio:
    #     def __init__(self):
    #         self.data = {
    #             'Wir': np.random.rand(50, nx + nu),
    #             'Wrr': np.random.rand(50, 50),
    #             'Wro': np.random.rand(ny, 51),
    #             'Wbr': np.random.rand(50, 1),
    #             'gama': 0.3
    #         }

    # modelo = ModeloPreditorFicticio()
    arquivo_modelo = 'weightsESNx_TR400_TVaz0.9_RaioE0.4.mat'
    caminho_arquivo_modelo = os.path.join(MODELS_FOLDER, arquivo_modelo)
    modelo = sio.loadmat(caminho_arquivo_modelo)
    
    # Crie uma matriz de vazão fictícia
    #matriz_simulador_vazao = np.random.rand(100, 3)  # Suponha 100 pontos com [Freq, Press, Vazao]

    # Chame a função InicializaSolver_limpo
    try:
        resultado = inicializa_solver(Hp, Hc, Qy, Qu, R, ny, nu, nx, modelo, matriz_simulador_vazao)
        print("inicializa_solver executou com sucesso!")
        print("Tipo de retorno:", type(resultado))
        print("Chaves do dicionário retornado:", resultado.keys())
        
        # Verifique se o solucionador foi criado corretamente
        if 'solucionador' in resultado:
            print("Solucionador criado com sucesso!")
            
            # Teste o solucionador com alguns dados fictícios
            # Nota: Isso é apenas um exemplo, você precisa adaptar para o seu caso específico
            x0 = np.random.rand(nx)
            p = np.random.rand(nx + nu + ny + nu + 50)  # 50 é o tamanho do reservatório da ESN no nosso exemplo
            
            sol = resultado['solucionador'](x0=x0, p=p)
            print("Solucionador executado com sucesso!")
            print("Forma da solução:", sol['x'].shape)
        else:
            print("Erro: Solucionador não encontrado no resultado.")
    
    except Exception as e:
        print(f"Erro ao executar inicializa_solver: {str(e)}")

if __name__ == "__main__":
    teste_InicializaSolver_limpo()