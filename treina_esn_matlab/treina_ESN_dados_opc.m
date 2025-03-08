clear;
close all;


nome_arquivo_dados= 'df_opc_mpa_10s.parquet';

% Configuração da semente aleatória
rng('default');
rng(42);

% Constantes para experimentar no treinamento
taxa_vazamento = 0.5;
raio_espectral = 0.5;
psi= 0; % 0.5;
in_scale = 0.01;%0.01;
bias_scale = 0.5;%0.5;
noise_amplitude = 0; %0.005;
warmupdrop=0;
warmup=1000;
tamanho_do_reservatorio = 100;
add_data_warmup = 1;

reg_min = 1e-8;
reg_max = 1e-4;

cv_folds = 5;

normalizar_v2=0;

plotar = 1;
salvar = 0;




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
n_in = length(variaveis_preditoras);  % 12 (número de variáveis de entrada)
n_out = length(variaveis_preditas);   % 10 (número de variáveis de saída)

desc_treino = 'estacionarios';% '072024';
desc_validacao = 'naoestacionarios';

% Definir períodos de interesse
periodos_interesse = {
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

% Período de validação
%periodo_validacao = {'2024-07-12 23:06:04', '2024-07-13 06:48:00'}; %meu
%DataHoraIni  ={'2024-07-12 10:00:00', '2024-07-16 23:00:00'}; %leizer
%DataHoraFim={'2024-07-12 16:50:00', '2024-07-17 02:30:00'}; %leizer
%periodo_validacao = {'2024-07-12 09:30:00', '2024-07-12 16:50:00'};  % alinhamento teste, inicio estacionario, freq e pmon subindo e descendo
%periodo_validacao2 = {'2024-07-16 22:00:00', '2024-07-17 05:00:00'};  % inicio estacionario, freq sobe, pmon desce  
%periodo_validacao3 = {'2024-09-09 00:00:00', '2024-09-09 13:20:00'}; % inicio nao estacionario, freq e pmon sobe

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


df=parquetread(nome_arquivo_dados);
df=fillmissing(df,'previous');


disp('Cria a ESN com parâmetros personalizados');

esn = ESN_warmupfix(tamanho_do_reservatorio, n_in, n_out, ...
    'gama', taxa_vazamento, ...        % taxa de vazamento
    'ro', raio_espectral, ...         % raio espectral
    'psi', psi, ...        % esparsidade
    'in_scale', in_scale, ...   % escala de entrada
    'noise_amplitude', noise_amplitude, ...
    'warmupdrop', warmupdrop,...
    'bias_scale', bias_scale); 




disp('Pré-processa dados para treinamento')
for i = 1:length(periodos_interesse)
    periodo = periodos_interesse{i};
    disp(periodo);
    
    % Seleciona dados do período
    idx_periodo = df.data_hora >= periodo{1} & df.data_hora <= periodo{2};
    df_periodo = df(idx_periodo, :);
    matriz_df = table2array(df_periodo(:, variaveis_preditoras)); 

    %normalizando
    for k=1:length(matriz_df)
        if normalizar_v2==1
            matriz_df(k, :) = normaliza_entradas_v2(matriz_df(k,:));
        else
            matriz_df(k, :) = normaliza_entradas(matriz_df(k,:));
        end

    end
    
    y_treina = matriz_df(2:end, 3:end);
    
    u_treina = matriz_df(1:end-1, :);
    
    % Adiciona dados ao ESN
    if add_data_warmup
        esn.add_data_warmup(u_treina, y_treina, warmup);
    else
        esn.add_data(u_treina, y_treina);
    end
end

disp('Treinando ESN...');
[erro_cv, melhor_reg] = esn.cum_train_cv(reg_min, reg_max, cv_folds);
fprintf('Melhor regularização: %.10f\n', melhor_reg);
fprintf('Erro CV: %.6f\n', erro_cv);

testa_esn_predicao_direta(esn, df, periodos_validacao{1}, variaveis_preditoras, variaveis_preditas, variaveis_apelidos_unidades, plotar, normalizar_v2);


n_periodos = length(periodos_validacao);
mapes = zeros(1, n_periodos);
for i=1:n_periodos
    periodo_validacao = periodos_validacao{i};
    mape_atual = testa_esn_multi_passos(esn, df, periodo_validacao, variaveis_preditoras, variaveis_preditas, variaveis_manipuladas, variaveis_apelidos_unidades, plotar, normalizar_v2);
    mapes(i) = mean(mape_atual);
end

% Testar ESN com previsão multi-passos
%mape1 = testa_esn_multi_passos(esn, df, periodo_validacao, variaveis_preditoras, variaveis_preditas, variaveis_manipuladas, variaveis_apelidos_unidades, plotar);

%mape2 = testa_esn_multi_passos(esn, df, periodo_validacao2, variaveis_preditoras, variaveis_preditas, variaveis_manipuladas, variaveis_apelidos_unidades, plotar);

%mape3 = testa_esn_multi_passos(esn, df, periodo_validacao3, variaveis_preditoras, variaveis_preditas, variaveis_manipuladas, variaveis_apelidos_unidades, plotar);
mape_testes = mean(mapes);
%mape_testes = mean([mape1, mape2, mape3]);

fprintf('\nMAPE msa testes: %.2f%%\n', mape_testes);

% Cria nome do modelo com os parâmetros
nome_arquivo_modelo = sprintf("weightsESNx_TR%d_TVaz%.2f_RaioE%.2f-treino_%s-mape_msa_%s_%.2f.mat", ...
    tamanho_do_reservatorio, taxa_vazamento, raio_espectral, desc_treino, desc_validacao, mape_testes);
disp(nome_arquivo_modelo)

% Salva o modelo
if salvar
    esn.save_reservoir(nome_arquivo_modelo);
end



%nome_arquivo_modelo_leizer = 'weightsESNx_TR400_TVaz0.9_RaioE0.4.mat';
%disp('Testando outra esn')
%disp(nome_arquivo_modelo_leizer)
%esn.load_reservoir(nome_arquivo_modelo_leizer);
%testa_esn(esn, df, periodo_validacao, variaveis_preditoras, variaveis_preditas, plotar);


