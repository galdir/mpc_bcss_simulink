function modelos = carregaModelosLSTM(pasta,mostra)

% Define the pasta onde estão os arquivos
%pasta = 'caminho/para/sua/pasta'; % substitua pelo caminho da sua pasta
% mostra =1/0 apenas na fase de depuração, para mostrar os resultados da busca na tela
% 
% Obtenha a lista de todos os arquivos na pasta
if nargin<2       % Mantém compatibilidade com versões anteriores
    mostra=0;    % Se não definou parâmetro, por padrão, não mostra
end

arquivos = dir(pasta);

% Inicialize a lista para os arquivos .mat
lista_nomes_modelos = {};
modelos = {};

% Itere sobre todos os arquivos e adicione os arquivos .mat à lista
for i = 1:length(arquivos)
    % Verifique se o arquivo tem extensão .mat
    if endsWith(arquivos(i).name, '.json')
        % Adicione o nome do arquivo .mat à lista
        [~, nome_arquivo, ~] = fileparts(arquivos(i).name);
        lista_nomes_modelos{end+1} = nome_arquivo;
    end
end

% Exiba a lista de arquivos .mat
if mostra
    disp('Arquivos de json de modelos encontrados:');
    disp(lista_nomes_modelos);
end

for i =1:length(lista_nomes_modelos)
    nome_modelo_lstm = lista_nomes_modelos{i};
    modelo = load(strcat(pasta, nome_modelo_lstm, '.mat'));
    modelo_lstm = modelo.net;

    % carrega configuracoes do modelo
    configuracoes_modelo_lstm = jsondecode(fileread(strcat(pasta, nome_modelo_lstm,'.json')));
    variaveisPreditoras = configuracoes_modelo_lstm.preditoras';
    variavelAlvo = configuracoes_modelo_lstm.variavel_predicao;
    amostrasPassadas = configuracoes_modelo_lstm.num_amostras_passadas;
    num_variaveis = size(variaveisPreditoras', 1);
    entradas_atuais = zeros(num_variaveis, amostrasPassadas);

    %salva modelo no conjunto de modelos
    modelos{i} = {variavelAlvo, modelo_lstm, configuracoes_modelo_lstm, entradas_atuais};
end



