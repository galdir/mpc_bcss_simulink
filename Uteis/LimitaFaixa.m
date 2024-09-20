function Valor=LimitaFaixa(ValorAtual,Faixa);  % Limita a variável nos limites definidos
    % Valor Atual é o valor atual da variável em análise
    % Faixa é um vetor com os limites [ Max  Min]  da faixa 
    Valor =min(ValorAtual,Faixa(1));          % Menor entre o ValorAtual e o Máximo
    Valor =max(ValorAtual,Faixa(2));         % Menor entre o ValorAtual e o Mínimo
end