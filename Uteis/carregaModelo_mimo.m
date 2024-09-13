function modelo = carregaModelo_mimo(pasta)

% Obtenha a lista de todos os arquivos na pasta
arquivos = dir(pasta);

% Inicialize a lista para os arquivos .mat
lista_nomes_modelos = {};

% Itere sobre todos os arquivos e adicione os arquivos .mat à lista
for i = 1:length(arquivos)
    % Verifique se o arquivo tem extensão .mat
    if endsWith(arquivos(i).name, '.json')
        % Adicione o nome do arquivo .mat à lista
        [~, nome_arquivo, ~] = fileparts(arquivos(i).name);
        lista_nomes_modelos{end+1} = nome_arquivo;
    end
end

if(isempty(lista_nomes_modelos))
    error('Nenhum modelo encontrado na pasta.');
end

if(length(lista_nomes_modelos)>1)
    error('Mais de um modelo encontrado na pasta.');
end


nome_modelo = lista_nomes_modelos{1};
modelo = load(strcat(pasta, nome_modelo, '.mat'));
modelo_lstm = modelo.net;

% carrega configuracoes do modelo
configuracoes_modelo_lstm = jsondecode(fileread(strcat(pasta, nome_modelo,'.json')));
variaveisPreditoras = configuracoes_modelo_lstm.preditoras';
variavelAlvo = configuracoes_modelo_lstm.variavel_predicao;
amostrasPassadas = configuracoes_modelo_lstm.num_amostras_passadas;
num_variaveis = size(variaveisPreditoras', 1);
entradas_atuais = zeros(num_variaveis, amostrasPassadas);

%modelo_lstm = load(strcat(pasta, 'mod_', variavelAlvo, '_casadi.mat'));

%salva modelo no conjunto de modelos
modelo = {variavelAlvo, modelo_lstm, configuracoes_modelo_lstm, entradas_atuais};
end



