function New = Interpola_casadi_vazao(Freq, Press, matriz)
% Interpola_casadi Realiza interpolação bilinear usando CasADi
%   Interpola valores baseados em Frequência e Pressão usando CasADi
%
%   Entradas:
%       Freq   - Valor de frequência para interpolar (objeto simbólico CasADi)
%       Press  - Valor de pressão para interpolar (objeto simbólico CasADi)
%       matriz - matrix contendo os dados de freq, press e vazao

%
%   Saída:
%       New - Struct contendo os valores interpolados ou valor específico se Indice for fornecido

import casadi.*


% Busca pontos da vizinhança para proceder a interpolação bilinear
Pontos = selPontos_casadi_vazao(Freq, Press, matriz);

% Inicializa New como uma struct
New = struct();

% Lista de campos para interpolação
%campos = {'VazaoOleo', 'VazaoLiquido', 'Twh', 'Pwh', 'DeltaP', 'PressSuccao'};

% Realiza a interpolação bilinear para cada campo

campo = 'VazaoOleo';
f1 = Pontos.FreqBCSS(1);
f2 = Pontos.FreqBCSS(3);
p1 = Pontos.PressChegada(1);
p2 = Pontos.PressChegada(2);
New.(campo) = Bilinear_casadi(Freq, Press, f1, f2, p1, p2, Pontos.(campo)(1), Pontos.(campo)(2), Pontos.(campo)(3), Pontos.(campo)(4));


New = New.(campo);


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
    %f_x = if_else(x1 == x2,((f12-f11)/(y2-y1))*y+(f11-((f12-f11)/(y2-y1))*y1), f);
    
    f_y = if_else(y1 == y2, f11 + (f21 - f11) * (x - x1) / (x2 - x1),  f);
    %f_y = if_else(y1 == y2, ((f22-f11)/(x2-x1))*x+f11-((f22-f11)/(x2-x1))*x1,  f);
    
    % Seleciona o resultado apropriado
    f = if_else(x1 == x2, f_x, if_else(y1 == y2, f_y, f));
    f = if_else(x1 == x2, if_else(y1 == y2, f11, f), f);

end