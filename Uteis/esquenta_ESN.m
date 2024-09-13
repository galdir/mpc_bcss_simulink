function proximo_estado = esquenta_ESN(modelo_esn, entradas_normalizadas,N)
% O parâmetro N permite a conta de atualização em um único passo ou em N
% passos, neste segundo caso, útil para "esquentar" a ESN na partida

leakrate = modelo_esn.gama;                             % Taxa de vazamento da ESN
estado_reservatorio_esn = modelo_esn.a0;     % Atual estado do reservatório da ESN

% Loop para atualizar estado do reservatório da ESN
for i=1:N
    % passo para calcular novos estados da ESN com base no estado atual e na taxa de vazamento
    xk_1 = modelo_esn.Wrr*estado_reservatorio_esn +  modelo_esn.Wir*entradas_normalizadas + modelo_esn.Wbr; 
    estado_reservatorio_esn = (1-leakrate)*estado_reservatorio_esn + leakrate*tanh(xk_1);
end
proximo_estado = estado_reservatorio_esn; 