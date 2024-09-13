function T=LeConverteNomes(NomeArq);

% Le a tabela com dados de simulação e já atualiza as colunas com os nomes das variáveis
% A tabela tem os dados fixos por coluna, assim, os nomes das colunas são ajustados para facilitar o código
% Não alteramos o cabeçalho da XLSX para não alterar os nomes originais
% criados pelo Simulador da Petrobras. Isso fará gerar um WARNING
% De qq modo, os nomes das variáveis (nas colunas da tabela) serão convertidos para facilitar o código

T1=readtable(NomeArq);
T=rmmissing(T1);               % Proteção - elimina linhas NaN
if height(T1)~=height(T)      % Alterou tamanho por terem sido estraidas linhas NaN
    disp(strcat("Extraidos NaN da Tabela original ",NomeArq))  % Indica na tela
end

% OBS: Não alteramos o cabeçalho original para não gerar dúvidas, trocamos
% os nomes apenas para facilitar o código
T.Properties.VariableNames(1)={'FreqBCSS'};           % Atualiza nome da coluna na tabela T
T.Properties.VariableNames(2)={'PressChegada'};    % Atualiza nome da coluna na tabela T
T.Properties.VariableNames(3)={'VazaoOleo'};          % Atualiza nome da coluna na tabela T
T.Properties.VariableNames(4)={'VazaoLiquido'};       % Atualiza nome da coluna na tabela T
T.Properties.VariableNames(5)={'Twh'};                       % Atualiza nome da coluna na tabela T
T.Properties.VariableNames(6)={'Pwh'};                       % Atualiza nome da coluna na tabela T
T.Properties.VariableNames(7)={'DeltaP'};                   % Atualiza nome da coluna na tabela T

% Cria coluna de Pressão de Sucção com base na Pressão de Descarga e no DeltaP
T.('PressSuccao')=T.Pwh-T.DeltaP;
