function New = Interpola_casadi_v2(Freq, Press, T, Indice)
% Interpola_casadi Realiza interpolação bilinear usando CasADi
%   Interpola valores baseados em Frequência e Pressão usando CasADi
%
%   Entradas:
%       Freq   - Valor de frequência para interpolar (objeto simbólico CasADi)
%       Press  - Valor de pressão para interpolar (objeto simbólico CasADi)
%       T      - Tabela contendo os dados
%       Indice - (Opcional) Índice específico para retornar
%
%   Saída:
%       New - Struct contendo os valores interpolados ou valor específico se Indice for fornecido

import casadi.*

% Converte a tabela para struct
T_struct = table2struct(T);

% Busca pontos da vizinhança para proceder a interpolação bilinear
Pontos = selPontos_casadi(Freq, Press, T_struct);

% Inicializa New como uma struct
New = struct();

% Lista de campos para interpolação
campos = {'VazaoOleo', 'VazaoLiquido', 'Twh', 'Pwh', 'DeltaP', 'PressSuccao'};

% Realiza a interpolação bilinear para cada campo
if nargin == 4
    campo = campos{Indice-2};
    f1 = Pontos.FreqBCSS(1);
    f2 = Pontos.FreqBCSS(3);
    p1 = Pontos.PressChegada(1);
    p2 = Pontos.PressChegada(2);
    New.(campo) = Bilinear_casadi(Freq, Press, f1, f2, p1, p2, Pontos.(campo)(1), Pontos.(campo)(2), Pontos.(campo)(3), Pontos.(campo)(4));
else
    for i = 1:length(campos)
        campo = campos{i};
        f1 = Pontos.FreqBCSS(1);
        f2 = Pontos.FreqBCSS(3);
        p1 = Pontos.PressChegada(1);
        p2 = Pontos.PressChegada(2);
        New.(campo) = Bilinear_casadi(Freq, Press, f1, f2, p1, p2, Pontos.(campo)(1), Pontos.(campo)(2), Pontos.(campo)(3), Pontos.(campo)(4));
    end
end

% Adiciona Freq e Press
New.FreqBCSS = Freq;
New.PressChegada = Press;

% Se um índice específico for solicitado
if nargin == 4
    %campos_ordenados = {'FreqBCSS', 'PressChegada', 'VazaoOleo', 'VazaoLiquido', 'Twh', 'Pwh', 'DeltaP', 'PressSuccao'};
    %valores = [];
    %for i = 1:length(campos_ordenados)
    %    valores = [valores; New.(campos_ordenados{i})];
    %end
    %New = valores(Indice);
    campo = campos{Indice-2};
    New = New.(campo);
end

end

function f = Bilinear_casadi(x, y, x1, x2, y1, y2, f11, f12, f21, f22)
% Bilinear_casadi Realiza interpolação bilinear usando CasADi
    import casadi.*
    
    % Interpolação BILINEAR
    K = (x2 - x1) * (y2 - y1);
    f = f11*(x2-x)*(y2-y) + f21*(x-x1)*(y2-y) + f12*(x2-x)*(y-y1) + f22*(x-x1)*(y-y1);
    f = f / K;
    
    % Casos especiais para interpolação LINEAR
    f_x = if_else(x1 == x2, f11 + (f12 - f11) * (y - y1) / (y2 - y1),  f);
    
    f_y = if_else(y1 == y2, f11 + (f21 - f11) * (x - x1) / (x2 - x1),  f);
    
    % Seleciona o resultado apropriado
    f = if_else(x1 == x2, f_x, if_else(y1 == y2, f_y, f));
end