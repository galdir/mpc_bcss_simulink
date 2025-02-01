classdef ESN < handle
    % ESN (Echo State Network) - Rede Neural de Estado de Eco
    % Esta classe implementa uma rede neural de estado de eco para modelagem de séries temporais
    
    properties
        neu             % número de neurônios
        n_in            % número de entradas
        n_out           % número de saídas
        psi             % fator de esparsidade
        leakrate        % taxa de vazamento
        ro              % raio espectral
        in_scale        % escala de entrada
        bias_scale      % escala do bias
        alfa            % condição inicial do algoritmo RLS
        forget          % fator de esquecimento
        output_feedback % feedback de saída
        noise           % amplitude do ruído
        
        % Matrizes de peso
        Wrr  % matriz de pesos do reservatório
        Wir  % matriz de pesos entrada-reservatório
        Wbr  % matriz de pesos bias-reservatório
        Wor  % matriz de pesos saída-reservatório
        Wro  % matriz de pesos reservatório-saída
        
        % Estado do reservatório
        a    % estado atual
        P    % matriz de covariância
        
        % Dados de treinamento
        cum_data_input        % dados de entrada acumulados
        cum_data_output       % dados de saída acumulados
        number_of_simulations % número de simulações
        simulations_start     % lista que rastreia cada início de simulação
        warmupdrop    % warmupdrop 
    end
    
    methods
        function obj = ESN(neu, n_in, n_out, varargin)
            % Construtor da ESN
            % Parâmetros:
            %   neu: número de neurônios
            %   n_in: número de entradas
            %   n_out: número de saídas
            %   Parâmetros opcionais (nome-valor):
            %     'gama': taxa de vazamento (default: 0.5)
            %     'ro': raio espectral (default: 1)
            %     'psi': esparsidade (default: 0.5)
            %     'in_scale': escala de entrada (default: 0.1)
            %     'bias_scale': escala do bias (default: 0.5)
            %     'alfa': valor inicial RLS (default: 10)
            %     'forget': fator de esquecimento (default: 1)
            %     'output_feedback': feedback de saída (default: false)
            %     'noise_amplitude': amplitude do ruído (default: 0)
            %     'out_scale': escala da saída (default: 0)
            %     'warmupdrop': amostras iniciais descartadas durante treino (default: 100)
            
            p = inputParser;
            addParameter(p, 'gama', 0.5);
            addParameter(p, 'ro', 1);
            addParameter(p, 'psi', 0.5);
            addParameter(p, 'in_scale', 0.1);
            addParameter(p, 'bias_scale', 0.5);
            addParameter(p, 'alfa', 10);
            addParameter(p, 'forget', 1);
            addParameter(p, 'output_feedback', false);
            addParameter(p, 'noise_amplitude', 0);
            addParameter(p, 'out_scale', 0);
            addParameter(p, 'warmupdrop', 100);
            
            parse(p, varargin{:});
            
            % Inicialização das propriedades básicas
            obj.neu = neu;
            obj.n_in = n_in;
            obj.n_out = n_out;
            obj.psi = p.Results.psi;
            
            % Inicialização das matrizes com distribuição normal
            obj.Wrr = obj.sparsidade(randn(neu, neu), obj.psi);
            obj.Wir = randn(neu, n_in);
            obj.Wbr = randn(neu, 1);
            obj.Wor = randn(neu, n_out);
            obj.Wro = randn(n_out, neu + 1);
            
            % Configuração dos parâmetros
            obj.leakrate = p.Results.gama;
            obj.ro = p.Results.ro;
            obj.in_scale = p.Results.in_scale;
            obj.bias_scale = p.Results.bias_scale;
            obj.alfa = p.Results.alfa;
            obj.forget = p.Results.forget;
            obj.output_feedback = p.Results.output_feedback;
            obj.noise = p.Results.noise_amplitude;
            
            % Normalização do raio espectral
            eigs_val = eig(obj.Wrr);
            radius = max(abs(eigs_val));
            obj.Wrr = obj.Wrr/radius * obj.ro;
            
            % Escalonamento das matrizes
            obj.Wbr = obj.bias_scale * obj.Wbr;
            obj.Wir = obj.in_scale * obj.Wir;
            obj.Wor = p.Results.out_scale * obj.Wor;
            
            % Inicialização do estado
            obj.a = zeros(neu, 1);
            
            % Matriz de covariância
            obj.P = eye(neu+1)/obj.alfa;
            
            % Variáveis de acumulação de dados
            obj.cum_data_input = [];
            obj.cum_data_output = [];
            obj.number_of_simulations = 0;
            obj.simulations_start = [];
            obj.warmupdrop = p.Results.warmupdrop;
        end
        
        function y = update(obj, inp, y_in, training)
            % Atualiza o estado da rede e retorna a saída
            % Parâmetros:
            %   inp: entrada da rede (n_in x 1)
            %   y_in: feedback da saída (opcional)
            %   training: modo de treinamento (opcional)
            
            % Verificação e preparação dos argumentos
            if nargin < 4
                training = false;
            end
            if nargin < 3 || all(y_in == 0)
                y_in = zeros(obj.n_out, 1);
            end
            
            % Garante formato correto dos dados
            inp = reshape(inp, [], 1);
            y_in = reshape(y_in, [], 1);
            
            if numel(inp) ~= obj.n_in
                error('A entrada deve ter tamanho n_in (%d), mas tem tamanho %d', obj.n_in, numel(inp));
            end
            
            % Cálculo do estado
            z = obj.Wrr * obj.a + obj.Wir * inp + obj.Wbr;
            if obj.output_feedback
                z = z + obj.Wor * y_in;
            end
            if obj.noise > 0 && training
                z = z + obj.noise * randn(obj.neu, 1);
            end
            
            % Atualização do estado
            obj.a = (1-obj.leakrate) * obj.a + obj.leakrate * tanh(z);
            
            % Cálculo da saída
            a_wbias = [1; obj.a];
            y = obj.Wro * a_wbias;
        end
        
        function add_data(obj, input_data, output_data, warmupdrop)
            % Adiciona dados de treinamento à rede
            % Parâmetros:
            %   input_data: matriz de dados de entrada (amostras x n_in)
            %   output_data: matriz de dados de saída (amostras x n_out)
            %   warmupdrop: número de amostras iniciais a descartar
            
            if nargin < 4
                warmupdrop = obj.warmupdrop;
            end
            
            % Verifica dimensões
            [n_samples, n_inputs] = size(input_data);
            if n_inputs ~= obj.n_in
                error('Número incorreto de entradas: esperado %d, recebido %d', obj.n_in, n_inputs);
            end
            
            % Registra início da nova simulação
            if isempty(obj.simulations_start)
                obj.simulations_start = 0;
            else
                obj.simulations_start(end+1) = size(obj.cum_data_input, 1);
            end
            
            % Coleta estados do reservatório
            A = zeros(n_samples-warmupdrop, obj.neu);
            for i = 1:n_samples
                %if obj.output_feedback && i > 1
                %    obj.update(input_data(i,:)', output_data(i-1,:)', true);
                %else
                    obj.update(input_data(i,:)', [], true);
                %end
                
                if i > warmupdrop
                    A(i-warmupdrop,:) = obj.a';
                end
            end
            
            % Acumula dados
            if isempty(obj.cum_data_input)
                obj.cum_data_input = A;
                obj.cum_data_output = output_data(warmupdrop+1:end,:);
            else
                obj.cum_data_input = [obj.cum_data_input; A];
                obj.cum_data_output = [obj.cum_data_output; output_data(warmupdrop+1:end,:)];
            end
            
            obj.number_of_simulations = obj.number_of_simulations + 1;
        end

        function add_data_warmup(obj, input_data, output_data, warmup)
            % Adiciona dados de treinamento à rede
            % Parâmetros:
            %   input_data: matriz de dados de entrada (amostras x n_in)
            %   output_data: matriz de dados de saída (amostras x n_out)
            %   warmup: número de simulacoes com a primeira amostra para
            %   esquentar o modelo para o ponto atual
            
            % Verifica dimensões
            [n_samples, n_inputs] = size(input_data);
            if n_inputs ~= obj.n_in
                error('Número incorreto de entradas: esperado %d, recebido %d', obj.n_in, n_inputs);
            end
            
            % Registra início da nova simulação
            if isempty(obj.simulations_start)
                obj.simulations_start = 0;
            else
                obj.simulations_start(end+1) = size(obj.cum_data_input, 1);
            end
            
            if warmup > 0
                for i = 1:warmup
                    obj.update(input_data(1,:)', [], true);
                end
            end

            % Coleta estados do reservatório
            A = zeros(n_samples, obj.neu);
            for i = 1:n_samples
                %if obj.output_feedback && i > 1
                %    obj.update(input_data(i,:)', output_data(i-1,:)', true);
                %else
                    obj.update(input_data(i,:)', [], true);
                %end
                
                A(i,:) = obj.a';

            end
            
            % Acumula dados
            if isempty(obj.cum_data_input)
                obj.cum_data_input = A;
                obj.cum_data_output = output_data(1:end,:);
            else
                obj.cum_data_input = [obj.cum_data_input; A];
                obj.cum_data_output = [obj.cum_data_output; output_data(1:end,:)];
            end
            
            obj.number_of_simulations = obj.number_of_simulations + 1;
        end
        
        function [best_error, best_reg] = cum_train_cv(obj, min_reg, max_reg, folds, tests)
            % Treina a rede usando validação cruzada
            % Parâmetros:
            %   min_reg: valor mínimo de regularização
            %   max_reg: valor máximo de regularização
            %   tests: número de testes (default: 50)
            %   folds: número de partições (default: 10)
            if nargin < 4
                folds = 10;
            end
            if nargin < 5
                tests = 50;
            end
            
            
            if min_reg > max_reg
                error('min_reg deve ser menor que max_reg');
            end
            
            reg_list = linspace(min_reg, max_reg, tests);
            n_samples = size(obj.cum_data_input, 1);
            val_size = floor(n_samples/folds);
            
            % Adiciona bias aos dados
            A_wbias = [ones(n_samples,1) obj.cum_data_input];
            
            best_error = inf;
            best_reg = 0;
            
            % Loop principal da validação cruzada
            for i = 1:length(reg_list)
                regularization = reg_list(i);
                fold_errors = zeros(1, folds);
                
                for fold = 1:folds
                    % Índices para validação
                    val_start = (fold-1)*val_size + 1;
                    if fold == folds
                        val_end = n_samples;
                    else
                        val_end = fold*val_size;
                    end
                    val_idx = val_start:val_end;
                    train_idx = setdiff(1:n_samples, val_idx);
                    
                    % Separa dados de treino e validação
                    training_A = A_wbias(train_idx,:);
                    training_y = obj.cum_data_output(train_idx,:);
                    cv_A = A_wbias(val_idx,:);
                    cv_y = obj.cum_data_output(val_idx,:);
                    
                    % Treina e avalia
                    theta = obj.reg_minimos_quadrados(training_A, training_y, regularization);
                    pred_y = cv_A * theta;
                    fold_errors(fold) = mean((cv_y - pred_y).^2, 'all');
                end
                
                mean_error = mean(fold_errors);
                if mean_error < best_error
                    best_error = mean_error;
                    best_reg = regularization;
                end
            end
            
            % Treinamento final com melhor regularização
            obj.Wro = obj.reg_minimos_quadrados(A_wbias, obj.cum_data_output, best_reg)';
        end
        
        function save_reservoir(obj, fileName)
            % Salva o reservatório em arquivo .mat
            % Parâmetros:
            %   fileName: nome do arquivo
            
            data = struct();
            data.Wrr = obj.Wrr;
            data.Wir = obj.Wir;
            data.Wbr = obj.Wbr;
            data.Wro = obj.Wro;
            data.a0 = obj.a;
            data.gama = obj.leakrate;
            save(fileName, 'data');
        end
        
        function load_reservoir(obj, fileName)
            % Carrega o reservatório de um arquivo .mat
            % Parâmetros:
            %   fileName: nome do arquivo
            
            data = load(fileName);
            obj.load_reservoir_from_struct(data.data);
        end
        
        function load_reservoir_from_struct(obj, data)
            % Carrega o reservatório a partir de uma estrutura
            % Parâmetros:
            %   data: estrutura com os dados do reservatório
            
            % Carrega matrizes principais
            if isfield(data, 'Wrr')
                obj.Wrr = data.Wrr;
            end
            if isfield(data, 'Wir')
                obj.Wir = data.Wir;
            end
            if isfield(data, 'Wbr')
                obj.Wbr = data.Wbr;
            end
            if isfield(data, 'Wro')
                obj.Wro = data.Wro;
            end
            
            % Carrega estado inicial se disponível
            if isfield(data, 'a0')
                obj.a = data.a0;
            end
            
            % Carrega bias se disponível
            if isfield(data, 'Wro_b')
                obj.Wro = [data.Wro_b obj.Wro];
            end
            
            % Carrega taxa de vazamento
            if isfield(data, 'gama')
                if isstruct(data.gama)
                    obj.leakrate = data.gama(1,1);
                else
                    obj.leakrate = data.gama;
                end
            end
            
            % Atualiza dimensões baseado nas matrizes carregadas
            obj.neu = size(obj.Wrr, 1);
            obj.n_in = size(obj.Wir, 2);
            obj.n_out = size(obj.Wro, 1);
        end
    end
    
    methods (Access = private)
        function M_esparsa = sparsidade(~, M, psi)
            % Cria uma matriz esparsa
            % Parâmetros:
            %   M: matriz de entrada
            %   psi: fator de esparsidade
            
            N = zeros(size(M));
            for linha = 1:size(N,1)
                for coluna = 1:size(N,2)
                    if rand < psi
                        N(linha,coluna) = 0;
                    else
                        N(linha,coluna) = 1;
                    end
                end
            end
            M_esparsa = N .* M;
        end
        
        function theta = reg_minimos_quadrados(~, X, Y, reg)
            % Calcula os mínimos quadrados regularizados
            % Parâmetros:
            %   X: matriz de características
            %   Y: matriz de saídas
            %   reg: parâmetro de regularização
            
            P = X' * X;
            R = X' * Y;
            theta = (P + reg*eye(size(P))) \ R;
        end

        % Versão 2: Usando SVD (ainda mais estável, mas mais lento)
        function theta = reg_minimos_quadrados_svd(~, X, Y, reg)
            % Calcula mínimos quadrados regularizados usando SVD
            % Esta versão é a mais estável numericamente, especialmente para matrizes mal condicionadas
            % X: matriz de características
            % Y: matriz de saídas
            % reg: parâmetro de regularização
            
            [U, S, V] = svd(X, 'econ');
            s = diag(S);
            
            % Fatores de regularização para cada valor singular
            f = s ./ (s.^2 + reg);
            
            % Calcula theta usando SVD
            theta = V * (f .* (U' * Y));
        end
    end
end