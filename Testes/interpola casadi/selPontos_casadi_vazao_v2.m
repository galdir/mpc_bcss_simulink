function Pontos = selPontos_casadi_vazao_v2(Freq, Press, matriz)
% selPontos_casadi Seleciona 4 pontos da tabela T para interpolação bilinear usando CasADi
%   Conhecido (Freq,Press), seleciona os 4 pontos da tabela T para proceder a interpolação Bilinear
%
%   Entradas:
%       Freq     - Valor de frequência para interpolar (objeto simbólico CasADi)
%       Press    - Valor de pressão para interpolar (objeto simbólico CasADi)
%       matriz - matrix contendo os dados de freq, press e vazao
%
%   Saída:
%       Pontos - Struct contendo os 4 pontos selecionados para interpolação

import casadi.*

% Extrai os vetores de Frequência e Pressão
FreqBCSS = matriz(:,1);
PressChegada = matriz(:,2);

% Obtém grids únicos para Frequência e Pressão
gridF = unique(FreqBCSS);
gridP = unique(PressChegada);

% Verifica e ajusta Frequência e Pressão se estiverem fora dos limites
Freq = verificarLimites(Freq, gridF);
Press = verificarLimites(Press, gridP);

% Encontra pontos de interpolação para Frequência e Pressão
[F1, F2] = encontrarPontosInterpolacao_v2(Freq, gridF);
[P1, P2] = encontrarPontosInterpolacao_v2(Press, gridP);

% Inicializa struct para armazenar os pontos
Pontos = struct();

% Obtém os nomes dos campos
%campos = fieldnames(T_struct);

%campo = 'VazaoOleo';
%valores = [T_struct.(campo)];
valores=matriz(:,3);

nP = length(gridP);
nF = length(gridF);

% Reorganiza os valores em uma matriz 2D
valores_2d = reshape(valores, nP, nF);

% Seleciona os 4 pontos para interpolação usando indexação simbólica
Pontos.VazaoOleo = [
    selecionarValor(valores_2d, gridP, gridF, P1, F1);
    selecionarValor(valores_2d, gridP, gridF, P2, F1);
    selecionarValor(valores_2d, gridP, gridF, P1, F2);
    selecionarValor(valores_2d, gridP, gridF, P2, F2)
];


% Adiciona Freq e Press selecionados
Pontos.FreqBCSS = [F1; F1; F2; F2];
Pontos.PressChegada = [P1; P2; P1; P2];

end

function valor = verificarLimites(valor, grid)
    import casadi.*
    
    minGrid = grid(1);
    maxGrid = grid(end);
    
    valor = if_else(valor < minGrid, minGrid, ...
             if_else(valor > maxGrid, maxGrid, valor));
end



function [inferior, superior] = encontrarPontosInterpolacao(valor, grid)
    import casadi.*
    
    n = length(grid);
    
    % Criar vetores de condições
    cond_lower = valor >= grid;
    cond_upper = valor < grid;
    
    % Combinar as condições
    cond_combined = cond_lower(1:end-1) & cond_upper(2:end);
    
    % Adicionar a condição para o último elemento
    cond_last = valor == grid(end);
    cond_combined = [cond_combined; cond_last];
    
    % Criar vetores para inferior e superior
    inferior_vec = [grid(1:end-1); grid(end-1)];
    superior_vec = grid;
    
    % Usar sum ponderada para selecionar os pontos corretos
    inferior = sum1(cond_combined .* inferior_vec);
    superior = sum1(cond_combined .* superior_vec);
end


function [inferior, superior] = encontrarPontosInterpolacao_v2(valor, grid)
    import casadi.*
    
    n = length(grid);
    
    % Inicializar inferior e superior
    inferior = grid(1);
    superior = grid(end);
    
    % Criar vetores de condições
    cond_lower = valor >= grid(1:end-1);
    cond_upper = valor < grid(2:end);
    
    % Combinar as condições
    cond_combined = cond_lower & cond_upper;
    
    % Usar sum1 ponderada para selecionar os pontos corretos
    inferior = sum1(cond_combined .* grid(1:end-1)) + if_else(valor == grid(end), grid(end-1), 0);
    superior = sum1(cond_combined .* grid(2:end)) + if_else(valor == grid(end), grid(end), 0);
    
    % Caso o valor seja exatamente igual ao primeiro ponto do grid
    inferior = if_else(valor == grid(1), grid(1), inferior);
    superior = if_else(valor == grid(1), grid(2), superior);
end

function valor = selecionarValor(matriz, gridP, gridF, P, F)
    import casadi.*
    
    valor = 0;
    for i = 1:length(gridP)
        for j = 1:length(gridF)
            cond = (gridP(i) == P) & (gridF(j) == F);
            valor = if_else(cond, matriz(i,j), valor);
        end
    end
end


function valor = selecionarValor_v2(matriz, gridP, gridF, P, F)
    import casadi.*
    
    % Criar as condições de igualdade para as dimensões P e F
    condP = gridP == P;
    condF = gridF == F;
    
    % Converter para vetores coluna e linha
    condP_col = condP(:);  % Garantir que condP seja uma coluna
    condF_row = condF(:)';  % Garantir que condF seja uma linha
    
    % Criar a matriz de condição combinada (produto externo das condições)
    cond_combined = condP_col * condF_row;
    
    % Usar sum1 para obter o valor correto na matriz com base nas condições
    valor = sum1(sum1(cond_combined .* matriz));
end