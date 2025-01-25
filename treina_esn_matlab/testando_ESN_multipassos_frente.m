clear;
close all;

tamanho_do_reservatorio = 100;
cv_folds = 5;

%nome_arquivo_modelo = 'weightsESNx_TR100_TVaz0.12_RaioE0.99_mape_msa_8809.59.mat';
nome_arquivo_modelo = 'weightsESNx_TR400_TVaz0.9_RaioE0.4_mape_msa_3.38.mat';
%nome_arquivo_modelo = 'weightsESNx_JUB27n100_mape_msa_4.85.mat';
%nome_arquivo_modelo = 'weightsESNx_TR300_TVaz0.8_RaioE0.1_mape_msa_3.41.mat';
%nome_arquivo_modelo = 'weightsESNx_TR200_TVaz0.50_RaioE0.99_mape_msa_2.62.mat';
%nome_arquivo_modelo = 'weightsESNx_TR100_TVaz0.50_RaioE0.99_mape_msa_2.44.mat';
%nome_arquivo_modelo = 'weightsESNx_TR100_TVaz0.90_RaioE0.99_mape_msa_4.02.mat';
%nome_arquivo_modelo = 'weightsESNx_TR900_TVaz0.9_RaioE0.4_mape_msa_3.12.mat';
%nome_arquivo_modelo = 'weightsESNx_TR100_TVaz0.50_RaioE0.99_mape_msa_2.35.mat';
%nome_arquivo_modelo = 'weightsESNx_TR200_TVaz0.50_RaioE0.99_mape_msa_dois_2.71.mat';

%nome_arquivo_modelo = 'weightsESNx_TR100_TVaz0.50_RaioE0.99.mat';

nome_arquivo_dados = 'df_opc_mpa_10s.parquet';
plotar = 1;

% Definição das variáveis
variaveis_manipuladas = {
    'frequencia_BCSS'
    'pressao_montante_alvo'
};

variaveis_preditas = {
    'pressao_succao_BCSS'
    'pressao_chegada'
    'pressao_diferencial_BCSS'
    'pressao_descarga_BCSS'
    'temperatura_motor_BCSS'
    'corrente_torque_BCSS'
    'corrente_total_BCSS'
    'temperatura_succao_BCSS'
    'vibracao_BCSS'
    'temperatura_chegada'
};

% Concatenar variáveis preditoras
variaveis_preditoras = [variaveis_manipuladas; variaveis_preditas];

% Definir as variáveis e suas unidades
variaveis_apelidos_unidades = {
    {0, 'P Succao', 'bar'}
    {1, 'P Chegada', 'bar'}
    {2, 'P Diferencial', 'bar'}
    {3, 'P Descarga', 'bar'}
    {4, 'T Motor', '°C'}
    {5, 'C Torque', 'A'}
    {6, 'C Total', 'A'}
    {7, 'T Succao', '°C'}
    {8, 'Vibracao', 'g'}
    {9, 'T Chegada', '°C'}
};

% Definir dimensões
n_in = length(variaveis_preditoras);
n_out = length(variaveis_preditas);

% Período de validação
periodo_validacao = {'2024-09-09 00:00:00', '2024-09-09 13:20:00'};
periodo_validacao2 = {'2024-07-12 23:06:04', '2024-07-13 06:48:00'};
%periodo_validacao = {'2024-09-09 00:00:00', '2024-09-09 00:20:00'};

% Carregar dados
df = parquetread(nome_arquivo_dados);
df = fillmissing(df, 'previous');

% Criar e carregar ESN
esn = ESN(tamanho_do_reservatorio, n_in, n_out);
disp(['Carregando modelo: ' nome_arquivo_modelo]);
esn.load_reservoir(nome_arquivo_modelo);

% Testar ESN com previsão multi-passos
mape1 = testa_esn_multi_passos(esn, df, periodo_validacao, variaveis_preditoras, variaveis_preditas, variaveis_manipuladas, variaveis_apelidos_unidades, plotar);

mape2 = testa_esn_multi_passos(esn, df, periodo_validacao2, variaveis_preditoras, variaveis_preditas, variaveis_manipuladas, variaveis_apelidos_unidades, plotar);
mape_testes = mean([mape1, mape2]);

fprintf('\nMAPE msa testes: %.2f%%\n', mape_testes);
