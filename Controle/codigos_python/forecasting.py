from logging.handlers import TimedRotatingFileHandler
import os
import scipy.io as sio

# from .forecaster import forecast
from datetime import datetime, timedelta
from time import time
import math
from typing import Tuple
import json
import numpy as np
import pandas as pd
from memory_profiler import profile
import logging

# os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
import tensorflow as tf
# Configurar o nível de log para suprimir mensagens
tf.get_logger().setLevel('ERROR')

from tensorflow import keras


# Obter o logger configurado no módulo principal
logger = logging.getLogger(__name__)


required_variables = [
    "frequencia",
    "choke",
    "pressao_succao",
    "pressao_chegada",
    "pressao_diferencial",
    "pressao_descarga",
    "temperatura_motor",
    "corrente_torque",
    "corrente_total",
    "temperatura_succao",
    "vibracao",
    "temperatura_chegada",
]


models_input = {
    "pressao_succao": ["frequencia", "choke", "pressao_succao", "temperatura_succao"],
    "pressao_chegada": [
        "frequencia",
        "choke",
        "pressao_succao",
        "pressao_chegada",
        "pressao_diferencial",
        "temperatura_succao",
        "temperatura_chegada",
    ],
    "pressao_diferencial": [
        "frequencia",
        "choke",
        "pressao_succao",
        "pressao_diferencial",
        "temperatura_succao",
    ],
    "pressao_descarga": [
        "frequencia",
        "choke",
        "pressao_succao",
        "pressao_diferencial",
        "temperatura_succao",
    ],
    "temperatura_motor": [
        "frequencia",
        "choke",
        "pressao_succao",
        "temperatura_motor",
        "corrente_torque",
        "temperatura_succao",
    ],
    "corrente_torque": [
        "frequencia",
        "choke",
        "pressao_succao",
        "corrente_torque",
        "temperatura_succao",
    ],
    "corrente_total": [
        "frequencia",
        "choke",
        "pressao_succao",
        "corrente_total",
        "temperatura_succao",
    ],
    "temperatura_succao": [
        "frequencia",
        "choke",
        "temperatura_succao",
    ],
    "vibracao": [
        "frequencia",
        "choke",
        "pressao_succao",
        "pressao_descarga",
        "corrente_torque",
        "temperatura_succao",
        "vibracao",
    ],
}


models_file_name = {
    "pressao_succao": "MSH_Out_ps_Inp_f_z_tsuc_fut_15_pass_30.h5",
    "pressao_chegada": "MSH_Out_pc_Inp_f_z_ps_pdif_tsuc_tche_fut_15_pass_30.h5",
    "pressao_diferencial": "MSH_Out_pdif_Inp_f_z_ps_tsuc_fut_15_pass_30.h5",
    "pressao_descarga": "MSH_Out_pdes_Inp_f_z_ps_tsuc_fut_15_pass_30.h5",
    "temperatura_motor": "MSH_Out_tm_Inp_f_z_ps_ctorq_tsuc_fut_15_pass_30.h5",
    "temperatura_succao": "MSH_Out_ts_Inp_f_z_fut_15_pass_30.h5",
    "corrente_torque": "MSH_Out_ctorq_Inp_f_z_ps_tsuc_fut_15_pass_30.h5",
    "corrente_total": "MSH_Out_ct_Inp_f_z_ps_tsuc_fut_15_pass_30.h5",
    "vibracao": "MSH_Out_vib_Inp_f_z_ps_pdes_ctorq_tsuc_fut_15_pass_30.h5",
}


def get_min_max_BCSS(coluna):
    # Definindo o máximo e mínimo para normalização
    min_freq = 0
    max_freq = 60

    min_abert = 0
    max_abert = 100

    min_psuccao = 0
    max_psuccao = 250

    min_pchegada = 0
    max_pchegada = 100

    min_pressao_diferencial_BCSS = 0
    max_pressao_diferencial_BCSS = 200

    min_pressao_descarga_BCSS = 0
    max_pressao_descarga_BCSS = 300

    min_PDG_pressao = 0
    max_PDG_pressao = 300

    min_temperatura_succao_BCSS = 0
    max_temperatura_succao_BCSS = 250

    min_temperatura_motor_BCSS = 0
    max_temperatura_motor_BCSS = 200

    min_PDG_temperatura = 0
    max_PDG_temperatura = 300

    min_vibracao_BCSS = 0
    max_vibracao_BCSS = 100

    min_temperatura_chegada = 0
    max_temperatura_chegada = 100

    min_header_psi_201a = 0
    max_header_psi_201a = 100

    min_corrente_total_BCSS = 0
    max_corrente_total_BCSS = 200

    min_corrente_torque_BCSS = 0
    max_corrente_torque_BCSS = 200

    min_tensao_saida_VSD = 0
    max_tensao_saida_VSD = 5000

    min_pressao_montante_alvo = 0
    max_pressao_montante_alvo = 100

    if coluna == "frequencia_BCSS":
        min_value = min_freq
        max_value = max_freq
    elif coluna == "choke_producao":
        min_value = min_abert
        max_value = max_abert
    elif coluna == "pressao_chegada":
        min_value = min_pchegada
        max_value = max_pchegada
    elif coluna == "pressao_succao_BCSS":
        min_value = min_psuccao
        max_value = max_psuccao
    elif coluna == "pressao_diferencial_BCSS":
        min_value = min_pressao_diferencial_BCSS
        max_value = max_pressao_diferencial_BCSS
    elif coluna == "pressao_descarga_BCSS":
        min_value = min_pressao_descarga_BCSS
        max_value = max_pressao_descarga_BCSS
    elif coluna == "PDG_pressao":
        min_value = min_PDG_pressao
        max_value = max_PDG_pressao
    elif coluna == "temperatura_succao_BCSS":
        min_value = min_temperatura_succao_BCSS
        max_value = max_temperatura_succao_BCSS
    elif coluna == "temperatura_motor_BCSS":
        min_value = min_temperatura_motor_BCSS
        max_value = max_temperatura_motor_BCSS
    elif coluna == "PDG_temperatura":
        min_value = min_PDG_temperatura
        max_value = max_PDG_temperatura
    elif coluna == "vibracao_BCSS":
        min_value = min_vibracao_BCSS
        max_value = max_vibracao_BCSS
    elif coluna == "temperatura_chegada":
        min_value = min_temperatura_chegada
        max_value = max_temperatura_chegada
    elif coluna == "header_psi_201a":
        min_value = min_header_psi_201a
        max_value = max_header_psi_201a
    elif coluna == "corrente_total_BCSS":
        min_value = min_corrente_total_BCSS
        max_value = max_corrente_total_BCSS
    elif coluna == "corrente_torque_BCSS":
        min_value = min_corrente_torque_BCSS
        max_value = max_corrente_torque_BCSS
    elif coluna == "tensao_saida_VSD":
        min_value = min_tensao_saida_VSD
        max_value = max_tensao_saida_VSD
    elif coluna == "pressao_montante_alvo":
        min_value = min_pressao_montante_alvo
        max_value = max_pressao_montante_alvo
    else:
        raise Exception(f"Coluna {coluna} desconhecida, não é possivel normalizar.")

    return min_value, max_value


def normalizar_dado_BCS(dado, coluna):
    min_value, max_value = get_min_max_BCSS(coluna)
    dado = (dado - min_value) / (max_value - min_value)
    return dado


def desnormalizar_dado_BCS(dado, coluna):
    min_value, max_value = get_min_max_BCSS(coluna)
    dado = (dado + min_value) * (max_value - min_value)
    return dado


def listar_pastas(diretorio):
    # Obtém uma lista de todos os arquivos e pastas no diretório especificado
    itens = os.listdir(diretorio)
    # Filtra apenas as pastas
    pastas = [item for item in itens if os.path.isdir(os.path.join(diretorio, item))]
    return pastas


def listar_arquivos_json(diretorio):
    # Obtém uma lista de todos os arquivos e pastas no diretório especificado
    itens = os.listdir(diretorio)
    # Filtra apenas os arquivos .json
    arquivos_json = [
        item
        for item in itens
        if item.endswith(".json") and os.path.isfile(os.path.join(diretorio, item))
    ]
    return arquivos_json


def get_LSTM_models_and_required_variables(MODELS_FOLDER, BASE_FOLDER):
    # print(MODELS_FOLDER)
    pastas = listar_pastas(MODELS_FOLDER)
    conjuntos_modelos = {}
    if len(pastas)==0:
        msg = f'Nenhuma pasta de modelos encontrada em {MODELS_FOLDER}'
        logger.error(msg)
        raise(msg)

    for pasta_conjunto_modelos in pastas:
        logger.debug(f'Carregando modelos {pasta_conjunto_modelos}')
        caminho_conjunto_modelos = os.path.join(MODELS_FOLDER, pasta_conjunto_modelos)
        # pasta_modelos = MODELS_FOLDER + '\\' + pasta
        # print(pasta_conjunto_modelos)
        arquivos_json = listar_arquivos_json(caminho_conjunto_modelos)
        conjunto_modelos = {}
        conjunto_modelos['lista_modelos'] = []

        if len(arquivos_json)==0:
            msg = f'Nenhum arquivo json encontrado em {pasta_conjunto_modelos}'
            logger.error(msg)
            raise(msg)

        for arquivo_json_modelo in arquivos_json:
            try:
                arquivo_json_modelo = os.path.join(
                    caminho_conjunto_modelos, arquivo_json_modelo
                )

                f = open(arquivo_json_modelo)
                dados_modelo = json.load(f)
                arquivo_modelo_keras = arquivo_json_modelo.split(".json")[0] + ".keras"
                if not os.path.exists(arquivo_modelo_keras):
                    arquivo_modelo_keras = arquivo_json_modelo.split(".json")[0] + ".h5"

                modelo = keras.models.load_model(arquivo_modelo_keras)
                dados_modelo["modelo_keras"] = modelo
                dados_modelo['buffer_modelo'] = pd.DataFrame()
                
                conjunto_modelos['lista_modelos'].append(dados_modelo)
                logger.debug(f'dados do modelo {dados_modelo["variavel_predicao"]} foram carregados')

                # dados_modelo = dados_modelo_BCSS(100, 'preco', ['variavel1', 'variavel2'], 'min_max', 'regressao', model)
            except Exception as e:
                logger.error(e)

        caminho_arquivo_preds = os.path.join(
                BASE_FOLDER, f"preds-modeloBCSS_{pasta_conjunto_modelos}.csv"
            )

        conjunto_modelos['caminho_arquivo_preds'] = caminho_arquivo_preds

         # Configuração do logger
        #caminho_logs_modelo = os.path.join(BASE_FOLDER, f'modelo_esn_{pasta_modelo}.log')
        nome_logger = cria_logger_modelo(pasta_conjunto_modelos, BASE_FOLDER)
        
        conjunto_modelos['nome_logger_modelo'] = nome_logger

        conjuntos_modelos[pasta_conjunto_modelos] = conjunto_modelos

    return conjuntos_modelos

def get_ESN_models_and_required_variables(MODELS_FOLDER, BASE_FOLDER):
    # print(MODELS_FOLDER)
    pastas = listar_pastas(MODELS_FOLDER)
    modelos = {}
    if len(pastas)==0:
        msg = f'Nenhuma pasta de modelos encontrada em {MODELS_FOLDER}'
        logger.error(msg)
        raise(msg)

    for pasta_modelo in pastas:
        logger.debug(f'Carregando modelo {pasta_modelo}')
        caminho_modelo = os.path.join(MODELS_FOLDER, pasta_modelo)
        # pasta_modelos = MODELS_FOLDER + '\\' + pasta
        # print(pasta_conjunto_modelos)
        arquivos_json = listar_arquivos_json(caminho_modelo)
        lista_modelos = []

        if len(arquivos_json)==0:
            msg = f'Nenhum arquivo json encontrado em {pasta_modelo}'
            logger.error(msg)
            raise(msg)
        
        if len(arquivos_json)>1:
            msg = f'Mais de um arquivo json encontrado na pasta de modelo único {pasta_modelo}'
            logger.error(msg)
            raise(msg)


        #for arquivo_json_modelo in arquivos_json:
        arquivo_json_modelo = arquivos_json[0]
        try:
            caminho_arquivo_json_modelo = os.path.join(
                caminho_modelo, arquivo_json_modelo
            )

            f = open(caminho_arquivo_json_modelo)
            dados_modelo = json.load(f)
            #print(dados_modelo)

            caminho_arquivo_modelo_mat = caminho_arquivo_json_modelo.split(".json")[0] + ".mat"
            modelo = sio.loadmat(caminho_arquivo_modelo_mat)

            #modelo = keras.models.load_model(caminho_arquivo_modelo_mat)
            dados_modelo["modelo_esn"] = modelo
            #dados_modelo['estado_modelo'] = pd.DataFrame()
            #lista_modelos.append(dados_modelo)
            logger.debug(f'dados do modelo esn {pasta_modelo} foram carregados')

            caminho_arquivo_preds = os.path.join(
                BASE_FOLDER, f"preds-modeloBCSS_{pasta_modelo}.csv"
            )

            dados_modelo['caminho_arquivo_preds'] = caminho_arquivo_preds

            
            # Configuração do logger
            #caminho_logs_modelo = os.path.join(BASE_FOLDER, f'modelo_esn_{pasta_modelo}.log')
            nome_logger = cria_logger_modelo(pasta_modelo, BASE_FOLDER)
            
            dados_modelo['nome_logger_modelo'] = nome_logger

            modelos[pasta_modelo] = dados_modelo

            # dados_modelo = dados_modelo_BCSS(100, 'preco', ['variavel1', 'variavel2'], 'min_max', 'regressao', model)
        except Exception as e:
            logger.error(e)

        
        

    return modelos

def cria_logger_modelo(pasta_modelo, BASE_FOLDER):
    # Configuração do logger
    nome_logger = f'logger_{pasta_modelo}'
    caminho_logs_modelo = os.path.join(BASE_FOLDER, f'log-modeloBCSS_{pasta_modelo}.log')
    
    logger_modelo_esn = logging.getLogger(nome_logger)
    logger_modelo_esn.setLevel(logging.DEBUG)
    handler_modelo_esn = TimedRotatingFileHandler(caminho_logs_modelo, when='midnight', interval=1, backupCount=7)
    #handler_logger_conjunto_modelos.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)s;%(name)s;%(levelname)s;%(message)s')
    handler_modelo_esn.setFormatter(formatter)
    logger_modelo_esn.addHandler(handler_modelo_esn)
    logger_modelo_esn.propagate = False

    return nome_logger


def is_sampling_interval_stable(arquivo_historico_variaveis, tempo_amostragem):
    lim_difference_inf = timedelta(seconds=tempo_amostragem * 0.8)  
    lim_difference_sup = timedelta(seconds=tempo_amostragem * 1.2)  
    try:
        df_historico = pd.read_csv(arquivo_historico_variaveis)

        if(len(df_historico)==1):
            return True

        data_hora_coleta_anterior = df_historico.iloc[-2]['data_hora']
        data_hora_coleta_anterior = datetime.strptime(data_hora_coleta_anterior, "%Y-%m-%d %H:%M:%S.%f")
        data_hora_coleta_atual = df_historico.iloc[-1]['data_hora']
        data_hora_coleta_atual = datetime.strptime(data_hora_coleta_atual, "%Y-%m-%d %H:%M:%S.%f")
        
        
        
        difference = data_hora_coleta_atual - data_hora_coleta_anterior
        if difference >= lim_difference_inf and difference <= lim_difference_sup:
            return True
        else:
            logger.error(f"Tempo de amostragem variou: {difference}s")
            return False
        
    except Exception as e:
        #print(f"Erro: {e}")
        logger.error(e)


def is_buffer_complete(arquivo_buffer, PAST_SAMPLES):
    try:
        f = open(arquivo_buffer, "r")
        lines = f.readlines()

        if len(lines) == PAST_SAMPLES + 1:
            return True

        return False

    except Exception as e:
        print(f"Erro: {e}")


def has_nan(variables_lst):
    try:
        for i in range(len(variables_lst)):
            variables_lst[i] = float(variables_lst[i])
            if math.isnan(variables_lst[i]):
                return True
        return False
    except ValueError:
        return True


def check_forecast_exceed_time(forecast_time, MAX_FORECAST_TIME):
    if forecast_time > MAX_FORECAST_TIME:
        return True
    return False


def read_input_file(
    file_name: str, dicionario_variaveis: dict
) -> Tuple[pd.DataFrame, str]:
    """
    Ordem esperada e tagas:
    frequencia_atual_bcs;temperatura_succao;pressao_succao;pressao_descarga;corrente_torque_bcs;vibracao_bcs;pressao_chegada;temperatura_chegada;choke;pressao_diferencial;temperatura_motor_bcs;corrente_total_bcs
    M54SI111E/1.PV_OS;M54TI105E/1.PV;M54PI103E/1.PV;M54PI104E/1.PV;M54II108E/1.PV;M54VXI107E/1.PV;T61PSI033/1.PV;T61TI035/1.PV;CHOKE-JUB-27;M54PDI109E/1.PV;M54TI106E/1.PV;M54IQI117E/1.PV

    ordem anterior:
    frequencia;choke;pressao_succao;pressao_chegada;pressao_diferencial;pressao_descarga;temperatura_motor;corrente_torque;corrente_total;temperatura_succao;vibracao;temperatura_chegada
    """
    try:
        with open(file_name, "r") as f:
            lines = f.readlines()
            #txt = lines[-1]
            #variables = txt.split(";")
            #NUMBER_OF_INPUT_VARIABLES = len(dicionario_variaveis)
            df_entrada = pd.DataFrame()
            if (len(lines) == 2):
                #status = "OK"
                df_entrada = pd.read_csv(file_name, sep=";")
                df_entrada.rename(columns=dicionario_variaveis, inplace=True)
                logger.debug(f'Arquivo de entrada carregado e com tags traduzidas')
            else:
                #status = "Problema no arquivo de entrada"
                logger.error(f'Problema no arquivo de entrada: {len(df_entrada)} linhas')

        return df_entrada
    except Exception as e:
        logger.error(e)
        print(f"Erro: {e}")



def run_forecast(
    modelo,
    required_variables,
    future_samples,
    df_entrada,
    total_forecast_time,
    MAX_FORECAST_TIME,
):

    start_time = time()
    forecast_result = forecast_with_future_manipulation(
        modelo, required_variables, future_samples, df_entrada
    )
    prediction_time = time() - start_time
    total_forecast_time += prediction_time

    if check_forecast_exceed_time(total_forecast_time, MAX_FORECAST_TIME):
        status = "Tempo de predicao excedido"
    else:
        status = None

    return status, forecast_result, total_forecast_time


def predicao_conjunto_modelos_lstm(
    dados_conj_modelos,
    df_buffer_periodo,
    nome_conj_modelos,
    MAX_FORECAST_TIME,
):
    total_forecast_time = 0
    nome_logger_modelo = dados_conj_modelos['nome_logger_modelo']
    logger_conjunto_modelos = logging.getLogger(nome_logger_modelo)
    logger_conjunto_modelos.debug('Iniciando predicoes.')
    lista_dados_modelos = dados_conj_modelos['lista_modelos']

    variaveis_alvo = [dados_modelo['variavel_predicao'] for dados_modelo in lista_dados_modelos]

    colunas_arquivo_predicao = ['data_hora', 'tempo_predicao'] + variaveis_alvo
    df_predicoes = pd.DataFrame([[np.nan] * len(colunas_arquivo_predicao)], columns=colunas_arquivo_predicao)
    #print(df_predicoes)

    estados_buffers = np.ones(len(variaveis_alvo))

    #atualizar buffers
    for i, dados_modelo in enumerate(lista_dados_modelos):

        #print(i)
        variavel_predicao = dados_modelo['variavel_predicao']
        #print(dados_modelo['variavel_predicao'])
        
        amostras_passadas = dados_modelo['num_amostras_passadas']

        atualiza_buffer_modelo(dados_modelo, df_buffer_periodo)
        if(len(dados_modelo["buffer_modelo"])==amostras_passadas):
            estados_buffers[i] = 1
        else:
            estados_buffers[i] = 0

    if estados_buffers.sum()==len(variaveis_alvo):
        logger_conjunto_modelos.debug(f'Buffers completos, iniciando predicoes.')
        for i, dados_modelo in enumerate(lista_dados_modelos):

            start_time = time()
            #print(i)
            variavel_predicao = dados_modelo['variavel_predicao']
            #print(dados_modelo['variavel_predicao'])
            
            amostras_passadas = dados_modelo['num_amostras_passadas']

            pred = predicao_modelo_lstm(dados_modelo, df_buffer_periodo)
            df_predicoes[variavel_predicao] = pred
            
            logger_conjunto_modelos.info(f'Predicao de {variavel_predicao}: {pred}')
            
            prediction_time = time() - start_time
            total_forecast_time += prediction_time
        
        logger_conjunto_modelos.info(f'Tempo total de predicao {total_forecast_time}')
        df_predicoes['tempo_predicao'] = total_forecast_time
        df_predicoes['data_hora'] = datetime.now()

        if df_predicoes.isna().sum().sum()==0:
            df_predicoes.to_csv(
                dados_conj_modelos['caminho_arquivo_preds'],
                mode="a",
                header=not pd.io.common.file_exists(dados_conj_modelos['caminho_arquivo_preds']),
                index=False,
            )
    else:
        logger_conjunto_modelos.info(f'Buffer do conjunto de modelos {nome_conj_modelos} em construção')
        
        #lista_dados_modelos[i] = dados_modelo
        
    
    #print(f'total_forecast_time: {total_forecast_time}')
    return df_predicoes

def predicao_modelo_ESN(
    dados_modelo,
    df_entrada: pd.DataFrame,
    nome_modelo,
    MAX_FORECAST_TIME,
):

    for i in range(len(df_entrada)):
        total_forecast_time = 0  

        start_time = time()
        nome_logger_modelo = dados_modelo['nome_logger_modelo']
        logger_modelo = logging.getLogger(nome_logger_modelo)
        logger_modelo.debug('Iniciando predicoes.')
        variaveis_alvo = dados_modelo['variaveis_predicao']
        variaveis_preditoras = dados_modelo['preditoras']

        entrada = df_entrada[variaveis_preditoras].iloc[i].to_numpy().reshape(-1)

        colunas_arquivo_predicao = ['data_hora', 'tempo_predicao'] + variaveis_alvo
        df_predicoes = pd.DataFrame([[np.nan] * len(colunas_arquivo_predicao)], columns=colunas_arquivo_predicao)

              

        pesos_reservatorio = dados_modelo['modelo_esn']['data']['Wrr'][0][0]
        pesos_entrada = dados_modelo['modelo_esn']['data']['Wir'][0][0]
        pesos_bias_reservatorio = dados_modelo['modelo_esn']['data']['Wbr'][0][0].reshape(-1)
        pesos_saida = dados_modelo['modelo_esn']['data']['Wro'][0][0]
        estado_reservatorio = dados_modelo['modelo_esn']['data']['a0'][0][0].reshape(-1)
        #gama = dados_modelo['modelo_esn']['data']['gama'][0][0]
        leakrate = dados_modelo['leakrate']

        if len(entrada) == pesos_entrada.shape[1]:

            for i,variavel in enumerate(variaveis_preditoras):
                entrada[i] = normalizar_dado_BCS(entrada[i], variavel)

            z = np.dot(pesos_reservatorio, estado_reservatorio) + np.dot(pesos_entrada, entrada) + pesos_bias_reservatorio
            novo_estado_reservatorio = (1 - leakrate) * estado_reservatorio + leakrate * np.tanh(z)
            dados_modelo['modelo_esn']['data']['a0'][0][0] = novo_estado_reservatorio
            
            a_wbias = np.hstack((1.0, novo_estado_reservatorio))
            y = np.dot(pesos_saida, a_wbias)
            #df_predicoes = pd.DataFrame(y).T
            #df_predicoes.columns = variaveis_alvo
            
            for i, variavel_alvo in enumerate(variaveis_alvo):
                pred = y[i]
                df_predicoes[variavel_alvo] = desnormalizar_dado_BCS(pred, variavel_alvo)
                logger_modelo.info(f'Predicao de {variavel_alvo}: {df_predicoes[variavel_alvo].values[0]}')

            prediction_time = time() - start_time
            total_forecast_time += prediction_time
                
            logger_modelo.info(f'Tempo total de predicao {total_forecast_time}')
            df_predicoes['tempo_predicao'] = total_forecast_time
            df_predicoes['data_hora'] = datetime.now()

            if df_predicoes.isna().sum().sum()==0:
                df_predicoes.to_csv(
                    dados_modelo['caminho_arquivo_preds'],
                    mode="a",
                    header=not pd.io.common.file_exists(dados_modelo['caminho_arquivo_preds']),
                    index=False,
                )
        else:
            logger_modelo.erro('Shape das variaveis de entrada diferente do shape do modelo.')

    return df_predicoes

def run_all_forecast(
    future_samples,
    arquivo_buffer,
    required_variables,
    modelos,
    MAX_FORECAST_TIME,
    NUMBER_OF_MODELS,
):
    total_forecast_time = 0
    forecasts = [None] * NUMBER_OF_MODELS
    df_entrada = pd.read_csv(arquivo_buffer, index_col=False)

    for i, modelo in enumerate(modelos):
        status, forecast_result, total_forecast_time = run_forecast(
            modelo,
            required_variables[i],
            future_samples,
            df_entrada,
            total_forecast_time,
            MAX_FORECAST_TIME,
        )
        forecasts[i] = forecast_result

        if status:
            return status, forecasts, total_forecast_time

    return None, forecasts, total_forecast_time


def atualiza_buffer_modelo(dados_modelo, df_entrada):
    amostras_passadas = dados_modelo['num_amostras_passadas']
    variaveis_preditoras = dados_modelo['preditoras']
    df_buffer = dados_modelo["buffer_modelo"]
    df_entrada_preditoras = df_entrada[variaveis_preditoras].copy()

    len_novas_amostras =  len(df_entrada_preditoras)

    if df_entrada.isna().sum().sum() > 0:
           msg = "Variaveis possuem valores NaN, os buffers vão ser reiniciados"
           logger.error(msg)
           df_buffer = pd.DataFrame()
    else:
        if len(df_buffer) == 0:
            df_buffer = df_entrada_preditoras.copy()
        else:
            # verificando se o buffer está incompleto
            if len(df_buffer) < amostras_passadas:
                df_buffer = pd.concat([df_buffer, df_entrada_preditoras], axis=0).reset_index(drop=True)

                if(len(df_buffer) > amostras_passadas):
                    diferenca_buffer = amostras_passadas - len(df_buffer)
                    df_buffer = df_buffer.drop(index=df_buffer.index[:diferenca_buffer]).reset_index(drop=True)
            else:
                df_buffer = df_buffer.shift(-len_novas_amostras) #deslocando os dados para tras para receber as novas amotras
                df_buffer.iloc[-len_novas_amostras:] = df_entrada_preditoras

    dados_modelo["buffer_modelo"] = df_buffer

@profile
def predicao_modelo_lstm(dados_modelo, df_entrada):

    variavel_alvo = dados_modelo['variavel_predicao']
    modelo_keras = dados_modelo['modelo_keras']
    amostras_passadas = dados_modelo['num_amostras_passadas']

    df_buffer = dados_modelo['buffer_modelo']

    pred = 0
    if(len(df_buffer)==amostras_passadas):
        df_buffer_norm = df_buffer.copy()
        for coluna in df_buffer.columns:
            df_buffer_norm[coluna] = normalizar_dado_BCS(df_buffer[coluna], coluna)

        X = np.asarray([df_buffer_norm.to_numpy()])
        #X = tf.convert_to_tensor(X, dtype=tf.float32)

        pred = modelo_keras.predict(X, verbose=0)
        tf.keras.backend.clear_session()
        pred = pred.reshape(-1)[0]

        # desnormalização
        pred = desnormalizar_dado_BCS(pred, variavel_alvo)
    else:
        raise('buffer incompleto')

    return pred


def forecast_with_future_manipulation(
    model, required_variables, future_samples, df_entrada
):
    # df_entrada = pd.read_csv(arquivo_buffer, index_col=False)
    for coluna in df_entrada.columns:
        normalizar_dado_BCS(df_entrada, coluna)

    df_entrada = df_entrada[required_variables].copy()

    # Repetindo a última linha para as amostras futuras
    for i in range(future_samples):
        df_entrada.loc[len(df_entrada)] = df_entrada.loc[len(df_entrada) - 1].values

    X = np.asarray([df_entrada.to_numpy()])
    

    # print(X)

    pred = model.predict(X)
    pred *= 100  # desnormalizando (despadronizando)

    pred = pred[0][0][0]
    # print(pred)

    return pred
