classdef NLPCallback < casadi.Callback
    methods
        function self = NLPCallback(name, opts)
            % Chamando o construtor da classe pai (casadi.Callback)
            self = self@casadi.Callback(name, opts);
        end

        function out = get_n_in(self)
            out = 3; % Número de entradas esperadas no callback
        end

        function out = get_n_out(self)
            out = 0; % Número de saídas esperadas no callback
        end

        function eval(self, varargin)
            % Código de avaliação do callback
            fval = varargin{1};
            x = varargin{2};
            
            % Verifica NaNs e imprime uma mensagem de depuração
            if any(isnan(full(x)))
                disp('NaN detectado em x durante a iteração.');
            end
            disp(['Função objetivo atual: ', num2str(full(fval))]);
        end
    end
end
