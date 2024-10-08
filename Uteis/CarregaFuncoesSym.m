% Carrega para área de trabalho algumas funções escritas na forma simbólica
% Serão consumidas pelo Solver, assim com, pelos códigos Matlab

% IMPORTANTE

% Para que possamos usar a mesma função em códigos Matlab e no CaSAdi,
% todas as funções terão no final do nome a terminação _sym
% Para obter a resposta de forma numérica, use o comando "full"

import casadi.*
Freq_sym = MX.sym('Freq_sym',1);        % Cria variável simbólica para a Frequencia
Press_sym = MX.sym('Press_sym',1);    % Cria variável simbólica para a Pressão

% Como estamos gereralizando, podemos usar esta variável simbólica em mais
% de um contexto ?????????????????????????????

%% ==========================================================
% Criando a função para, dada uma frequencia, buscar os limites dinâmicos
% (MAX/MIN) de todas as variáveis em função da frequência
% Lembrar que a busca é feita baseada em uma TabelaLimitesDinamicos pré-estabelecida, 
% a qual gerou na área de trabalho a MatrizLimitesDinamicos 
%
% Exemplo:
% Dado uma Freq_real, para chamar no Matlab, use Limites=full(f_buscaLimites_sym(Freq_real));
buscaLimitesMatriz_casadi_sym = buscaLimitesMatriz_casadi(MatrizLimitesDinamicos, Freq_sym);
f_buscaLimites_sym = Function('BuscaLimites', {Freq_sym}, {buscaLimitesMatriz_casadi_sym});

%% ==========================================================
% Criando a função para interpolação da vazão com base na tabela do Simulador Petrobras
% Para favorecer o desempenho computacional, note que carregou apenas as
% 3 primeiras colunas, com a informação de Freq, PChegada e Vazão, já que o
% objetivo é apenas estimar a vazão e não as demais variáveis
% AVALIAR SE NÃO VALE A PENA FAZER A INTERPOLAÇÃO DE TODA A LINHA E NÃO SÓ DA VAZÃO

Interpola_casadi_vazao_sym = Interpola_casadi_vazao(Freq_sym, Press_sym, MatrizSimulador(:,1:3));
f_Interpola_casadi_vazao_sym = Function('f_vazao', {Freq_sym, Press_sym}, {Interpola_casadi_vazao_sym});

