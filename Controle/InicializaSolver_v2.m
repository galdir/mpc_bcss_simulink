function solverMPC = InicializaSolver_v2(tipoModelo, horizPred, horizCont, pesoSaidas, pesoAlvos, pesoEntradas, numSaidas, numEntradas, numEstados, modeloPreditor)
% InicializaSolver - Inicializa o solver para otimização MPC
%
% Entradas:
%   tipoModelo    - Indica o tipo de modelo usado para o preditor MPC (1=ESN)
%   horizPred     - Horizonte de predição
%   horizCont     - Horizonte de controle
%   pesoSaidas    - Matriz diagonal para ponderação das variáveis controladas
%   pesoAlvos     - Matriz diagonal para ponderação dos alvos desejados
%   pesoEntradas  - Matriz diagonal para ponderação das variáveis manipuladas
%   numSaidas     - Número de variáveis controladas (2: PSuc e PChegada)
%   numEntradas   - Número de variáveis manipuladas (2: Freq e PMonAlvo)
%   numEstados    - Número de variáveis coletadas do processo (10)
%   modeloPreditor - Rede utilizada como preditor interno ao controlador

% Nomes originais:
% EstruturaSolver, Hp, Hc, Qy, Qu, R, ny, nu, nx, ModeloPreditor

import casadi.*

%% Define parâmetros simbólicos gerais (Casadi) para o problema de otimização
estadosPreditos = MX.sym('estadosPreditos', numEstados, horizPred+1);  % X
incrementosControle = MX.sym('incrementosControle', horizCont*numEntradas, 1);  % du
sequenciaControle = [incrementosControle; zeros(numEntradas*(horizPred-horizCont), 1)];  % Du
setpointOtimo = MX.sym('setpointOtimo', numSaidas, 1);  % ysp
variaveis_MPC = MX.sym('variaveis_MPC', numEstados);  % VarControladas
funcaoSaida = Function('funcaoSaida', {variaveis_MPC}, {variaveis_MPC(1:2)});  % h

%% Configuração do preditor ESN
if tipoModelo ~= 1
    error('Tipo de modelo inválido. Esta implementação suporta apenas ESN (tipoModelo == 1).');
end

tamanhoReservatorioESN = length(modeloPreditor.data.Wir);  % nx_ESN
parametrosSolver = MX.sym('parametrosSolver', numEstados+numEntradas+numSaidas+numEntradas+tamanhoReservatorioESN);  % P
entradasAtuais = parametrosSolver(numEstados+1:numEstados+numEntradas);  % uk_1
erroPredicao = parametrosSolver(numEstados+numEntradas+1:numEstados+numEntradas+numSaidas);  % erro
alvoRTO = parametrosSolver(numEstados+numEntradas+numSaidas+1:numEstados+numEntradas+numSaidas+numEntradas);  % uRTO
estadoInicialESN = parametrosSolver(numEstados+numEntradas+numSaidas+numEntradas+1:end);  % ESNdataa0

restricoes = [estadosPreditos(:,1) - parametrosSolver(1:numEstados)];  % g

%% Define a função objetivo e as restrições de forma recursiva ao longo do horizonte de predição
funcaoObjetivo = 0;  % fob
for k = 1:horizPred
    entradasAtuais = entradasAtuais + sequenciaControle((k-1)*numEntradas+1:k*numEntradas);
    saidasMedidas = funcaoSaida(estadosPreditos(:,k+1));  % ym
    funcaoObjetivo = funcaoObjetivo + ...
        (saidasMedidas-setpointOtimo+erroPredicao)'*pesoSaidas*(saidasMedidas-setpointOtimo+erroPredicao) + ...
        incrementosControle'*pesoEntradas*incrementosControle + ...
        (entradasAtuais-alvoRTO)'*pesoAlvos*(entradasAtuais-alvoRTO);
    
    entradasNormalizadas = normaliza_entradas([entradasAtuais; parametrosSolver(1:numEstados)]);  % ukk
    estadoESN = modeloPreditor.data.Wrr*estadoInicialESN + modeloPreditor.data.Wir*entradasNormalizadas + modeloPreditor.data.Wbr;  % x_ESN
    proximoEstadoESN = (1-modeloPreditor.data.gama)*estadoInicialESN + modeloPreditor.data.gama*tanh(estadoESN);  % next_state
    estadoComBias = [1.0; proximoEstadoESN];  % a_wbias
    saidasESN = modeloPreditor.data.Wro*estadoComBias;  % yn
    predicoesESN = desnormaliza_predicoes(saidasESN);  % y_esn_pred
    
    restricoes = [restricoes; estadosPreditos(:,k+1) - predicoesESN];
    estadoInicialESN = proximoEstadoESN;
end

%% Define as matrizes auxiliares para o incremento de controle
matrizM = [];  % Mtil
matrizI = [];  % Itil
matrizAuxiliar = zeros(numEntradas, horizCont*numEntradas);  % auxM
for i = 1:horizCont
    matrizAuxiliar = [eye(numEntradas) matrizAuxiliar(:, 1:(horizCont-1)*numEntradas)];
    matrizM = [matrizM; matrizAuxiliar];
    matrizI = [matrizI; eye(numEntradas)];
end
restricoes = [restricoes; matrizM*incrementosControle + matrizI*parametrosSolver(numEstados+1:numEstados+numEntradas)];

%% Configuração do otimizador
variaveisOtimizacao = [estadosPreditos(:); incrementosControle; setpointOtimo];  % opt_variable
problemaOtimizacao = struct('f', funcaoObjetivo, 'x', variaveisOtimizacao, 'g', restricoes, 'p', parametrosSolver);  % nlp

opcoesSolver = struct();
opcoesSolver.print_time = 0;
opcoesSolver.ipopt.print_level = 1;
opcoesSolver.ipopt.max_iter = 100;
opcoesSolver.ipopt.acceptable_tol = 1e-4;
opcoesSolver.ipopt.acceptable_obj_change_tol = 1e-4;

solverMPC = nlpsol('solverMPC', 'ipopt', problemaOtimizacao, opcoesSolver);  % SolucaoOtimizador
end