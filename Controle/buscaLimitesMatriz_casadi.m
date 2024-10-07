function lim = buscaLimitesMatriz_casadi(matriz, valorProcurado)
    import casadi.*
    n = size(matriz, 1);  % Número de linhas na coluna
    n2 = size(matriz, 2);  % Número de colunas
    %indices = MX.zeros(2, 1);
    contador = MX.zeros(1);
    limMax = MX.zeros(1, n2);
    limMin = MX.zeros(1, n2);
    for i = 1:n
        condicao = (matriz(i,1) == valorProcurado) & (contador < 2);
        limMax = if_else(condicao & (contador == 0), matriz(i,:), limMax);
        limMin = if_else(condicao & (contador == 1), matriz(i,:), limMin);
        contador = contador + if_else(condicao, 1, 0);
    end
    lim = [limMax(2:end); limMin(2:end)];
end