function Pontos = selPontos_v2(Freq, Press, T)
% selPontos Seleciona 4 pontos da tabela T para interpolação bilinear
%   Conhecido (Freq,Press), seleciona os 4 pontos da tabela T para proceder a interpolação Bilinear
%
%   Entradas:
%       Freq  - Valor de frequência para interpolar
%       Press - Valor de pressão para interpolar
%       T     - Tabela contendo os dados
%
%   Saída:
%       Pontos - Matriz 4x(largura de T) contendo os pontos selecionados para interpolação

    % Inicializa saída
    %Pontos = table();

    % Obtém grids únicos para Frequência e Pressão
    gridF = unique(T.FreqBCSS);
    gridP = unique(T.PressChegada);

    % Verifica e ajusta Frequência se estiver fora dos limites
    Freq = verificarLimites(Freq, gridF, 'Frequência', 'Hz');

    % Verifica e ajusta Pressão se estiver fora dos limites
    Press = verificarLimites(Press, gridP, 'Pressão de Chegada', 'Kgf/cm2');

    % Encontra pontos de interpolação para Frequência
    [F1, F2] = encontrarPontosInterpolacao(Freq, gridF);

    % Encontra pontos de interpolação para Pressão
    [P1, P2] = encontrarPontosInterpolacao(Press, gridP);

    % Extrai os 4 pontos para interpolação
    Pontos = [
        T(T.FreqBCSS == F1 & T.PressChegada == P1, :);
        T(T.FreqBCSS == F1 & T.PressChegada == P2, :);
        T(T.FreqBCSS == F2 & T.PressChegada == P1, :);
        T(T.FreqBCSS == F2 & T.PressChegada == P2, :)
    ];
end

function valor = verificarLimites(valor, grid, nome, unidade)
    if valor < min(grid) || valor > max(grid)
        warning('%s %.1f %s está fora da faixa de interpolação [%.0f, %.0f] %s', ...
            nome, valor, unidade, min(grid), max(grid), unidade);
        valor = max(min(grid), min(valor, max(grid)));
        warning('Cálculo feito com base em %s = %.0f %s', nome, valor, unidade);
    end
end

function [inferior, superior] = encontrarPontosInterpolacao(valor, grid)
    if ismember(valor, grid)
        inferior = valor;
        superior = valor;
    else
        idx = find(grid > valor, 1);
        inferior = grid(idx - 1);
        superior = grid(idx);
    end
end