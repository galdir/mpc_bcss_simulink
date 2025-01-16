% Script de Teste para a Função cria_solver
clear
import casadi.*;
CarregaTabelas; 
CarregaFuncoesSym;
[XIni,UIni]=SelCondicaoInicial('2024-07-17 01:00:00',MatrizSimulador);  

conversao_bar_kgf = 1.019716;
% Definir valores para os parâmetros de entrada

FreqMaxMin=[60 ,  40];                                                  % Limites máx/min para ser dado pelo controlador como entrada de Freq no processo
PMonAlvoMaxMin=[50 , 20];                                          % Limites máx/min para ser dado pelo controlador como entrada de PMon no processo
umax  = [FreqMaxMin(1) ,  PMonAlvoMaxMin(1)];       % Vetor com valor máximo das manipuladas (Freq e PMonAlvo)
umin  =  [FreqMaxMin(2), PMonAlvoMaxMin(2)] ;        % Vetor com valor mínimo das manipuladas  (Freq e PMonAlvo)
dumax = [0.1 , 1];                                                       %Variação máxima nas manipuladas [ Hz    bar ]

MargemPercentual = 1;

Hp = 3;
Hc = Hp-1;

Qy=  diag([1  10]);           % Qy - Peso das saidas controladas por setpoint = PChegada e Vazao)
Qu = diag([10  1]);              % Qu - Peso das ações de controle nas entradas (Alvos Desejados em  Freq. e PMonAlvo)
Qx= 0*diag(ones(1,11));    % Peso para os erros de estimação das  variáveis do processo
R=    0*diag([1  1]);             % R - Peso na variação das ações de controle - DeltaU em Freq. e PMonAlvo

nx=height(Qx);  % Número de variáveis (estados) do processo
nu=height(Qu);  % Número de variáveis de entrada no processo (manipuladas)
ny=height(Qy); % Número de variáveis de saida controladas por SetPoint

ESN_MPC = load('weightsESNx_TR400_TVaz0.9_RaioE0.4.mat');
ModeloPreditor = ESN_MPC;
%esquentar o modelo preditor esn (o solver da unfeasible sem esquentar)
entradas_normalizadas = normaliza_entradas([UIni;XIni]);   % Normaliza entradas provenientes do processo (observar que a função nada faz com a vazão)
for i=1:1000     % Esquenta a ESN
    % Novos estados da ESN com base no estado atual e na taxa de vazamento
    Predicao = ModeloPreditor.data.Wrr*ModeloPreditor.data.a0 +  ModeloPreditor.data.Wir*entradas_normalizadas + ModeloPreditor.data.Wbr;
    ModeloPreditor.data.a0 = (1-ModeloPreditor.data.gama)*ModeloPreditor.data.a0 + ModeloPreditor.data.gama*tanh(Predicao);
end

matriz_h=zeros(2,nx); % Tamanho da matriz que vai indicar as variáveis controladas por setpoint
matriz_h(1,2)=1;            % PChegada - Coluna na linha 1 que indica a primeira variável controlada
matriz_h(2,11)=1;          % Vazao - Coluna na linha 2  que indica a segunda variável controlada
estados_medidos_sym=MX.sym('estados_medidos',nx,1);
funcao_h=Function('h',{estados_medidos_sym},{matriz_h * estados_medidos_sym}); % Função de saída que mapeia diretamente o estado para a saída

WallTime = 9; % Tempo máximo de execução

% Chamar a função para criar o solver
[solver, args] = cria_solver(umax, umin, dumax, MargemPercentual, ...
    Hp, Hc, Qy, Qu, R, Qx, nx, nu, ny, ...
    f_Interpola_casadi_vazao_sym, f_buscaLimites_sym, ModeloPreditor, funcao_h, WallTime);

% Testar se o solver foi criado
if isa(solver, 'casadi.Function')
    disp('Solver criado com sucesso');
else
    disp('Erro na criação do solver');
end

opts_gen = struct('verbose', true,...
              'mex', true);
disp('Gerando codigo c');
solver.generate('solver_gen.c',opts_gen);
disp('Compilando dll');
% !C:\MinGW64\mingw64\bin\gcc.exe -v -fPIC -shared solver_gen.c -o solver_gen.dll -I"C:\Users\galdir\Documents\GitHub\mpc_bcss_simulink\Casadi\include" -L"C:\Users\galdir\Documents\GitHub\mpc_bcss_simulink\Casadi" -lipopt
% !C:\MinGW64\mingw64\bin\gcc.exe -fPIC -shared solver_gen.c -o solver_gen.dll
% !gcc.exe -fPIC -shared solver_gen.c -o solver_gen.dll
% 
% solver_compilado = external('solver', './solver_gen.dll');




%C = Importer('solver_gen.c','gcc');
%solver_gen = external('f',C);
%disp(f(3.14))


% Testar se o solver foi criado
if isa(solver, 'casadi.Function')
    disp('Solver compilado carregado com sucesso');
else
    disp('Erro na criação do solver');
end

% Verificar os argumentos do solver (args)
% disp('Limites inferiores para variáveis de decisão (lbx):');
% disp(args.lbx);
% 
% disp('Limites superiores para variáveis de decisão (ubx):');
% disp(args.ubx);
% 
% disp('Limites inferiores para restrições (lbg):');
% disp(args.lbg);
% 
% disp('Limites superiores para restrições (ubg):');
% disp(args.ubg);

%% testando o solver


x0=XIni;
X0_Hp=repmat(x0,1+Hp,1);                % Condição incial das variáveis medidas (estados X) atuais e futuras
u0=UIni;
U0_Hp=repmat(u0,Hp,1);                % Condição inicial para as ações de controle (U) em todo o horizonte Hp futuro

FreqAlvoIni=60;

Limites= full(f_buscaLimites_sym(FreqAlvoIni)); 
PMonAlvoIni=max([ Limites(2,2), PMonAlvoMaxMin(2)]);     % Mais conservador entre limite minimo (linha 2) da PChegada (coluna 2) ou a PMonAlvoMin definida

AlvoEng = [FreqAlvoIni; PMonAlvoIni];
Ysp= [ AlvoEng(2) ;    full(f_Interpola_casadi_vazao_sym(AlvoEng(1),AlvoEng(2)*conversao_bar_kgf)) ];

ErroX=zeros(nx,1);
ErroY=zeros(ny,1);
BuffDeltaFreq=zeros(45,1);

du0_Up=zeros(Hp*nu,1);          % Inicializa valores futuros com zeros (serão variáveis de decisão tratadas por restrição de igualdade)
solver_X0=[ X0_Hp;  U0_Hp; du0_Up];

args.p=[x0; u0; AlvoEng; Ysp; ErroX; ErroY; BuffDeltaFreq; ModeloPreditor.data.a0];

disp('Testando solver criado')
tic
solucao = solver('x0', solver_X0,'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
toc

feasible=solver.stats.success;
%iteracoes=solver.stats.iter_count;
disp('feasible:')
disp(feasible)
%disp('iteracoes:')
%disp(iteracoes)
solucao_manipuladas = full(solucao.x);
solucao_manipuladas = solucao_manipuladas(nx*(Hp+1)+1:nx*(Hp+1)+1+1);
disp(solucao_manipuladas)

disp('Testando solver compliado')
tic
%mex solver_gen.c -largeArrayDims % Matlab
mex -v -DMATLAB_MEX_FILE -I"C:\Users\luis.lima\Documents\mpc_bcss_simulink\Casadi\include" -L"C:\Users\luis.lima\Documents\mpc_bcss_simulink\Casadi" -lipopt solver_gen.c % Octave
solucao = solver_gen('solver', 'x0', solver_X0,'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
toc

solucao_manipuladas = full(solucao.x);
solucao_manipuladas = solucao_manipuladas(nx*(Hp+1)+1:nx*(Hp+1)+1+1);
disp(solucao_manipuladas)