function Pontos = selPontos_casadi_vazao(Freq, Press, matriz)
    % selPontos_casadi Seleciona 4 pontos da tabela T para interpolação bilinear usando CasADi
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
    FreqBCSS = matriz(:,1);
    PressChegada = matriz(:,2);

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

    % Seleciona os 4 pontos para interpolação usando indexação simbólica
    Pontos.VazaoOleo = [
        selecionarValor(matriz, P1, F1);
        selecionarValor(matriz, P2, F1);
        selecionarValor(matriz, P1, F2);
        selecionarValor(matriz, P2, F2);
        ];

    % Adiciona Freq e Press selecionados
    Pontos.FreqBCSS = [F1; F1; F2; F2];
    Pontos.PressChegada = [P1; P2; P1; P2];

end

function valor = verificarLimites(valor, grid)
    import casadi.*

    minGrid = grid(1);
    maxGrid = grid(end);

    valor = if_else(valor < minGrid, minGrid, if_else(valor > maxGrid, maxGrid, valor));
end

function [inferior, superior] = encontrarPontosInterpolacao(valor, grid)
    % essa funcao busca um par de valores para interpolar
    % caso o valor exista no no grid ela retorna o mesmo valor no par
    % caso nao exista ela retorna o par em torno do valor
    % caso seja um valor menor do que existe no grid, retorna o menor valor
    % do grid para o par
    % caso seja um valor maior do que existe no grid, retorna o maior valor
    % do grid para o par
    import casadi.*

    n = length(grid);
    inferior = grid(1);
    superior = grid(1);

    for i = 1:n
        cond = (valor == grid(i));
        inferior = if_else(cond, grid(i), inferior);
        superior = if_else(cond, grid(i), superior);
    end

    for i = 1:n-1
        cond = (valor > grid(i)) & (valor < grid(i+1));
        inferior = if_else(cond, grid(i), inferior);
        superior = if_else(cond, grid(i+1), superior);
    end

    cond = (valor > grid(end));
    inferior = if_else(cond, grid(end), inferior);
    superior = if_else(cond, grid(end), superior);

    cond = (valor < grid(1));
    inferior = if_else(cond, grid(1), inferior);
    superior = if_else(cond, grid(1), superior);

end

function valor = selecionarValor(matriz, P, F)
    import casadi.*

    %valor = 0;
    valor = matriz(1,3);
    for i = 1:size(matriz,1)
        cond = (matriz(i,1) == F) & (matriz(i,2) == P);
        valor = if_else(cond, matriz(i,3), valor);
    end
end