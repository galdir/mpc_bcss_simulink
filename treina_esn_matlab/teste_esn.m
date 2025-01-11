% Script de teste para a classe ESN
% Gera uma série temporal e usa a ESN para previsão

% Limpa o workspace
clear all;
close all;
clc;

%% Gera dados sintéticos

ruido = 0.2;

t = 0:0.1:50;  % vetor de tempo
y = sin(0.4*t) + ruido*randn(size(t));  % série temporal = seno + ruído
data = y';  % converte para coluna

%% Prepara dados para treino e teste
N = length(data);
train_len = floor(0.7 * N);  % 70% para treino
test_len = N - train_len;    % 30% para teste

% Organiza dados para entrada/saída (previsão um passo à frente)
X = data(1:end-1);  % entradas
Y = data(2:end);    % saídas (alvos)

% Separa conjuntos de treino e teste
X_train = X(1:train_len);
Y_train = Y(1:train_len);
X_test = X(train_len+1:end-1);
Y_test = Y(train_len+1:end-1);

% Vetores de tempo correspondentes
t_train = t(1:train_len);
t_test = t(train_len+1:end-1);

%% Cria e configura a ESN
n_neurons = 100;     % número de neurônios no reservatório
n_inputs = 1;        % dimensão da entrada
n_outputs = 1;       % dimensão da saída

% Cria a ESN com parâmetros personalizados
esn = ESN(n_neurons, n_inputs, n_outputs, ...
    'gama', 0.5, ...        % taxa de vazamento
    'ro', 0.9, ...         % raio espectral
    'psi', 0.1, ...        % esparsidade
    'in_scale', 0.1, ...   % escala de entrada
    'noise_amplitude', 1e-5); % ruído pequeno para estabilidade

%% Treina a ESN
warmupdrop = 50;  % descarta primeiros estados para estabilizar

% Adiciona dados de treino
esn.add_data(X_train, Y_train, warmupdrop);

% Treina usando validação cruzada
disp('Treinando ESN...');
[erro_cv, melhor_reg] = esn.cum_train_cv(1e-8, 1e-2, 20, 5);
fprintf('Melhor regularização: %.6f\n', melhor_reg);
fprintf('Erro CV: %.6f\n', erro_cv);

%% Testa a ESN
disp('Testando ESN...');
Y_pred = zeros(size(X_test));

% Reset do estado para teste
esn.a = zeros(n_neurons, 1);

% Fase de aquecimento usando últimos dados do treino
warmup_data = X_train(end-warmupdrop:end);
for i = 1:length(warmup_data)
    esn.update(warmup_data(i));
end

% Previsão
for i = 1:length(X_test)
    Y_pred(i) = esn.update(X_test(i));
end

% Calcula erro de teste
erro_teste = mean((Y_test - Y_pred).^2);
fprintf('Erro de teste (MSE): %.6f\n', erro_teste);

%% Visualização
figure;
subplot(2,1,1);
plot(t_train(1:length(Y_train)), Y_train, 'b-', 'LineWidth', 1);
title('Dados de Treino');
xlabel('Tempo');
ylabel('Amplitude');
grid on;

subplot(2,1,2);
hold on;
plot(t_test(1:length(Y_test)), Y_test, 'b-', 'LineWidth', 1, 'DisplayName', 'Real');
plot(t_test(1:length(Y_pred)), Y_pred, 'r--', 'LineWidth', 1, 'DisplayName', 'Previsto');
title('Previsão vs Real (Teste)');
xlabel('Tempo');
ylabel('Amplitude');
legend('show');
grid on;
hold off;

% Salva o reservatório treinado
esn.save_reservoir('esn_teste.mat');

% Teste de carregamento
esn_nova = ESN(n_neurons, n_inputs, n_outputs, ...
    'gama', 0.5, ...        % mesmos parâmetros da ESN original
    'ro', 0.9, ...
    'psi', 0.1, ...
    'in_scale', 0.1, ...
    'noise_amplitude', 1e-5);
esn_nova.load_reservoir('esn_teste.mat');

%% Teste adicional: previsão livre (geração autônoma)
n_previsoes = 100;
entrada_inicial = X_test(end);
previsoes = zeros(n_previsoes, 1);
entrada_atual = entrada_inicial;

% Reset do estado
esn_nova.a = zeros(n_neurons, 1);

% Fase de aquecimento
warmup_data = X_test(end-warmupdrop:end);
for i = 1:length(warmup_data)
    esn_nova.update(warmup_data(i));
end

% Geração autônoma
for i = 1:n_previsoes
    previsao = esn_nova.update(entrada_atual);
    previsoes(i) = previsao;
    entrada_atual = previsao;  % usa previsão como próxima entrada
end

% Plotar previsão livre
figure;
t_hist = t(end-100:end-1);
data_hist = data(end-100:end-1);
t_prev = (t(end-1):0.1:t(end-1)+0.1*n_previsoes)';

plot(t_hist(1:length(data_hist)), data_hist, 'b-', 'LineWidth', 1);
hold on;
plot(t_prev(1:length(previsoes)), previsoes, 'r--', 'LineWidth', 1);
title('Previsão Livre');
xlabel('Tempo');
ylabel('Amplitude');
legend('Histórico', 'Previsão');
grid on;