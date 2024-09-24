% SUGESTÃO:
% Deixe este arquivo aberto no seu ambiente do Matlab
% Ao antrar no Matlab, peça para rodar e o sistema vai pedir o ChangePath. 
% Com isso o ambiente se desloca para o diretório onde está o seu AjustaPath.m 
% Dai vc ajustas as primeiras linhas deste código, apontando para as pastas na sua máquina

% Aponta para diretórios de uso geral que estão em pastas do GitHub
addpath('.\Tabelas');             % Aponta para onde estão as Tabelas XLS Petrobras
addpath('.\Modelos\ESN');    % Aponta para a pasta onde estão os modelos de redes utilizadas no projeto
addpath('.\Uteis');                  % Aponta pasta com funcionalidade de utilidade geral consumida por vários
addpath('.\Casadi');               % Aponta para a pasta com as funções da biblioteca Casadi na versão utilziada pelo projeto

% Aponta para diretórios de uso geral que NÃO ESTÃO em pastas do GitHub
addpath('..\Dados');              % Aponta para onde estão os arquivos parquet com dados da Petrobras

% Se quiser, já desloca para a pasta em uso corrente
cd .\Controle
% cd ..\Mapas
