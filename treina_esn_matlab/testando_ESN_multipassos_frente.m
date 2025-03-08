clear;
close all;

tamanho_do_reservatorio = 100;
cv_folds = 5;

%nome_arquivo_modelo = 'weightsESNx_TR400_TVaz0.9_RaioE0.4.mat';
%nome_arquivo_modelo = 'weightsESNx_JUB27n100_mape_msa_4.85.mat';
nome_arquivo_modelo = 'weightsESNx_TR300_TVaz0.8_RaioE0.1.mat';
%nome_arquivo_modelo = 'weightsESNx_TR900_TVaz0.9_RaioE0.4_mape_msa_3.12.mat';
%nome_arquivo_modelo = 'weightsESNx_TR100_TVaz0.50_RaioE0.99_mape_msa_2.35.mat';
%nome_arquivo_modelo = 'weightsESNx_TR200_TVaz0.50_RaioE0.99_mape_msa_dois_2.71.mat';
%nome_arquivo_modelo = 'weightsESNx_TR200_TVaz0.50_RaioE0.50_mape_3msa1.71.mat';
%nome_arquivo_modelo = 'weightsESNx_TR200_TVaz0.50_RaioE0.50-treino_070809102024-mape_msa_070809102024_1.77.mat';
%nome_arquivo_modelo = 'weightsESNx_TR200_TVaz0.50_RaioE0.50-treino_072024-mape_msa_0809102024_2.04.mat';
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
%periodo_validacao = {'2024-07-12 23:06:04', '2024-07-13 06:48:00'}; %meu
%DataHoraIni  ={'2024-07-12 10:00:00', '2024-07-16 23:00:00'}; %leizer
%DataHoraFim={'2024-07-12 16:50:00', '2024-07-17 02:30:00'}; %leizer
%periodo_validacao = {'2024-07-12 10:00:00', '2024-07-12 16:50:00'}; % aceleracao e desaceleracao, alinhamento de teste
%periodo_validacao2 = {'2024-07-16 23:00:00', '2024-07-17 02:30:00'}; %leizer
%periodo_validacao3 = {'2024-09-09 00:00:00', '2024-09-09 13:20:00'}; %meu

% periodos_validacao = {
%     {'2024-07-12 10:00:00', '2024-07-12 16:50:00'}, % leizer
%     {'2024-07-16 23:00:00', '2024-07-17 02:30:00'}, %leizer
%     {'2024-09-09 00:00:00', '2024-09-09 13:20:00'}, %meu
%     {'2024-09-21 07:00:00', '2024-09-21 13:55:00'}, %meu
% };

periodos_validacao = {
    {'2024-07-12 09:30:00', '2024-07-12 16:50:00'}  % alinhamento teste, inicio estacionario, freq e pmon subindo e descendo
    {'2024-07-12 23:10:00', '2024-07-13 02:41:00'}  % inicio nao estacionario, freq e pmon subindo
    {'2024-07-13 03:30:00', '2024-07-13 06:48:00'}  % inicio estacionario, freq descendo
    {'2024-07-15 13:33:30', '2024-07-15 22:00:00'}  % inicio nao estacionario, freq pmon sobe
    {'2024-07-16 22:00:00', '2024-07-17 05:00:00'}  % inicio estacionario, freq sobe, pmon desce  
    {'2024-08-13 19:00:00', '2024-08-13 21:30:00'}  % inicio estacionario, freq sobe, pmon desce  
    {'2024-09-21 07:00:00', '2024-09-21 13:55:00'} % inicio estacionario, freq desce, pmon sobe  
    {'2024-09-09 00:00:00', '2024-09-09 13:20:00'} % inicio nao estacionario, freq e pmon sobe
    {'2024-09-21 22:30:00', '2024-09-22 05:10:00'} % inicio nao estacionario, freq e pmon sobe  
    {'2024-10-25 00:00:00', '2024-10-25 18:00:00'} % inicio estacionario, freq e pmon sobe e desce
};

% Carregar dados
df = parquetread(nome_arquivo_dados);
df = fillmissing(df, 'previous');

% Criar e carregar ESN
esn = ESN_warmupfix(tamanho_do_reservatorio, n_in, n_out);
disp(['Carregando modelo: ' nome_arquivo_modelo]);
esn.load_reservoir(nome_arquivo_modelo);

% Testar ESN com previsão multi-passos

n_periodos = length(periodos_validacao);
mapes = zeros(1, n_periodos);
for i=1:n_periodos
    periodo_validacao = periodos_validacao{i};
    mape_atual = testa_esn_multi_passos(esn, df, periodo_validacao, variaveis_preditoras, variaveis_preditas, variaveis_manipuladas, variaveis_apelidos_unidades, plotar);
    mapes(i) = mean(mape_atual);
end

%mape2 = testa_esn_multi_passos(esn, df, periodo_validacao2, variaveis_preditoras, variaveis_preditas, variaveis_manipuladas, variaveis_apelidos_unidades, plotar);

%mape3 = testa_esn_multi_passos(esn, df, periodo_validacao3, variaveis_preditoras, variaveis_preditas, variaveis_manipuladas, variaveis_apelidos_unidades, plotar);


%mape_testes = mean([mape1, mape2, mape3]);
mape_testes = mean(mapes);

fprintf('\nMAPE msa testes: %.2f%%\n', mape_testes);
