function Pontos = selPontos_casadi(Freq, Press, T_struct)
%   selPontos_casadi Seleciona 4 pontos da tabela T para interpolação bilinear usando CasADi
%   Conhecido (Freq,Press), seleciona os 4 pontos da tabela T para proceder a interpolação Bilinear
%
%   Entradas:
%       Freq     - Valor de frequência para interpolar (objeto simbólico CasADi)
%       Press    - Valor de pressão para interpolar (objeto simbólico CasADi)
%       T_struct - Struct contendo os dados da tabela (resultado de table2struct)
%
%   Saída:
%       Pontos - Struct contendo os 4 pontos selecionados para interpolação

import casadi.*

% Extrai os vetores de Frequência e Pressão
FreqBCSS = [T_struct.FreqBCSS];
PressChegada = [T_struct.PressChegada];

% Obtém grids únicos para Frequência e Pressão
gridF = unique(FreqBCSS);
gridP = unique(PressChegada);

% Verifica e ajusta Frequência e Pressão se estiverem fora dos limites
Freq = verificarLimites(Freq, gridF);
Press = verificarLimites(Press, gridP);

% Encontra pontos de interpolação para Frequência e Pressão
[F1, F2] = encontrarPontosInterpolacao(Freq, gridF);
[P1, P2] = encontrarPontosInterpolacao(Press, gridP);

% Inicializa struct para armazenar os pontos
Pontos = struct();

% Obtém os nomes dos campos
campos = fieldnames(T_struct);

% Para cada campo, seleciona os 4 pontos necessários para interpolação
for i = 1:length(campos)
    campo = campos{i};
    valores = [T_struct.(campo)];
    
    % Reorganiza os valores em uma matriz 2D
    valores_2d = reshape(valores, length(gridP), length(gridF));
    
    % Seleciona os 4 pontos para interpolação usando indexação simbólica
    Pontos.(campo) = [
        selecionarValor(valores_2d, gridP, gridF, P1, F1);
        selecionarValor(valores_2d, gridP, gridF, P2, F1);
        selecionarValor(valores_2d, gridP, gridF, P1, F2);
        selecionarValor(valores_2d, gridP, gridF, P2, F2)
    ];
end

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
    inferior = grid(1);
    superior = grid(end);
    
    for i = 1:n-1
        cond = (valor >= grid(i)) & (valor < grid(i+1));
        inferior = if_else(cond, grid(i), inferior);
        superior = if_else(cond, grid(i+1), superior);
    end
    
    % Tratamento especial para o último ponto do grid
    cond_last = (valor == grid(end));
    inferior = if_else(cond_last, grid(end-1), inferior);
    superior = if_else(cond_last, grid(end), superior);
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