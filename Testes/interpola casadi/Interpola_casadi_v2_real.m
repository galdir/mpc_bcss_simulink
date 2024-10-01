function result = Interpola_casadi_v2_real(Freq_real, Press_real, T, Indice)
% Interpola_casadi_real Wrapper para usar Interpola_casadi com valores reais
%   Esta função permite usar Interpola_casadi com valores reais de frequência e pressão
%
%   Entradas:
%       Freq_real  - Valor real de frequência para interpolar
%       Press_real - Valor real de pressão para interpolar
%       T          - Tabela contendo os dados
%       Indice     - (Opcional) Índice específico para retornar
%
%   Saída:
%       result - Struct contendo os valores interpolados ou valor específico se Indice for fornecido

import casadi.*

% Cria variáveis simbólicas CasADi
Freq_sym = SX.sym('Freq');
Press_sym = SX.sym('Press');

% Chama Interpola_casadi com variáveis simbólicas
New_sym = Interpola_casadi_v2(Freq_sym, Press_sym, T, Indice);

% Cria uma função CasADi
if isstruct(New_sym)
    % Caso em que New_sym é uma struct (sem Indice especificado)
    campos = fieldnames(New_sym);
    outputs = {};
    for i = 1:length(campos)
        outputs{end+1} = New_sym.(campos{i});
    end
    f = Function('f', {Freq_sym, Press_sym}, outputs);
    
    % Avalia a função com os valores reais
    valores = f(Freq_real, Press_real);
    
    % Constrói a struct de saída
    result = struct();
    for i = 1:length(campos)
        result.(campos{i}) = full(valores{i});
    end
else
    % Caso em que New_sym é um valor único (com Indice especificado)
    f = Function('f', {Freq_sym, Press_sym}, {New_sym});
    result = full(f(Freq_real, Press_real));
end

end