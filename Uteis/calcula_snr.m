function snr_db = calcula_snr(sinal)                % Estima uma relação sinal ruido com base na média e variância do sinal
    media = mean(sinal);                                    %  Media do sinal para ser usada como referência
    variancia = var(sinal);                                    % Variância do sinal
    snr = zeros(size(variancia));                        % Inicializa vetor com zeros
    snr(variancia ~= 0) = (media(variancia ~= 0).^2) ./ variancia(variancia ~= 0);   % Pontos em que a variância não é nula
    snr(variancia == 0) = Inf;                               % Pontos em que a variância é nula, assume SNR infinito
    snr_db = 10 * log10(snr);                              % 
end