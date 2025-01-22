function [T,Detalhe,DIni,DFim]=extrai_faixa2(NomeArq,DIni,DFim);

% Função para extrair faixa especifica de acordo com a tabela indicada
T = parquetread(NomeArq);
T.data_hora = datetime(T.data_hora, 'InputFormat', 'yyyy-MM-dd HH:mm:ss'); % Convertendo para o formato adequado

Detalhe='';                         % Só para inicializar a variável e cobrir os casos de NaN
Inicio = datetime(DIni);     % Usa o limite inicial definido
Fim = datetime(DFim);      % Usa o limite final definido
T = T(T.data_hora >= Inicio & T.data_hora <= Fim, :);    % Extrai os valores da Tabela apenas na faixa de tempo definida
 if height(T)==0
    Detalhe="Possivelmente selecionou na tabela XLS uma faixa de tempo inexistente no arquivo Parquet";
    beep
else
    % Indica valores efetivos de inicio e fim em função dos registros encontrados
    DIni=T.data_hora(1);
    DFim=T.data_hora(end);
    Detalhe=strcat("Periodo de ",datestr(DIni)," à ",datestr(DFim));
 end
disp(Detalhe)     % Só para mostrar o texto na tela




%===============================================================================