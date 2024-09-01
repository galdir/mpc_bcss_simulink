function T=ProtecaoDinamica(Freq,T,FxPercent);
% Função para calculo dos limites de alarmes de algumas variáveis. Estes não são valores
% fixos, mas sim em função da frequência de operação. Assim, esta função recebe a
% frequencia de operação e calcula os limites de proteção de algumas variáveis.
%
% Para isso, usaremos as tabelas fornecidas pela Petrobras como exemplo de Campo dos Goytacazes
% Uma vez mantido o formato, as tabelas XLSX pode ser substituida por novas tabelas (novas referências)

%=============================================================================
% Baseados no modelo da planilha de Proteção dinâmica do CAMPO DOS GOYTACAZES
% VARIÁVEIS QUE DEPENDEM DA FREQUÊNCIA e GERAM TRIP
% 1 - Production Surface Pressure   (Pressão de Chegada)
% 2 - Intake Pressure (Pressão de Sucção da BCSS)
% 3 - Discharge Pressure (Pressão de Descarga da BCSS)
% 4 - Differential Pressure (Pressão Diferencial da BCSS)
% 5 - Total Current (Corrente Total Motor BCSS)
% 6 - Torque Current (Corrente de Torque Motor BCSS)

% VARIÁVEIS QUE DEPENDEM DA FREQUÊNCIA mas NÃO SE APLICAM (NÃO USAMOS):
% Downhole Pressure (Pressão no fundo do poço)
% XTreePressure (Pressão na árvore de Natal) 

% VARIÁVEIS QUE DEPENDEM DA FREQUÊNCIA mas NÃO GERAM TRIP (MAS USAMOS):
% Production Surface Temperature (Temperatura de Chegada)
% Considerando que é uma informação que pode ser usada pelo controlador,
% optamos por incluir no hall das variáveis preditas a Temperatura de Chegada

%=============================================================================
% OUTRAS RESTRIÇÕES FIXAS QUE NÃO EXISTEM NA TABELA DE PROTEÇÃO DINÂMICA,
%   VISTA NO MODELO DE CAMPO DOS GOYTACAZES, MAS SIM NA MATRIZ DE CAUSA E EFEITO:
% 1 - Temperatura do Motor da BCSS (gera TRIP)
% 2 - Vibração (NÃO gera TRIP)
% 3 - Temperatura de succão (NÃO gera TRIP) 
%
% Falta definir forma de colocar estes limites fixos numa tabela para ser
% consumida pelo controlador. Talvez criar novas colunas na tabela atual ... AVALIAR !!!
% 
% Há outros critérios de proteção, a exemplo da variação brusca da
% corrente. Casos como estes, porém, não são percebidos pelo modelo mas já
% fazem parte das rotinas de proteção implementadas na empresa

if nargin<1      % Não definiu argumento (só na fase de depuração)
    Freq=40.4;
end
if nargin<3       % Não passou as duas tabelas como parâmetro
    % Carrega as tabelas de referências para as proteções dinâmicas
    addpath('..\Dados');
    T=readtable('DP.xlsx');                                 % Tabela de valores de referência para as variáveis
    FxPercent=readtable('DP-Faixas.xlsx');     % Tabela das faixas percentuais para disparar os alarmes LL, L, H e HH
end

% Há algumas proteções, pois, o valor da frequencia de operação não
% necessariamente está na tabela. Pode estar, inclusive, fora faixa coberta pela tabela
if Freq<min(T.FreqBCSS)       % Se o valor de frequencia for menor que o mínimo da tabela
    Freq=min(T.FreqBCSS);     % Assume o mínimo da tabela
end
if Freq>max(T.FreqBCSS)       % Se o valor de frequencia for maior que o maximo da tabela
    Freq=max(T.FreqBCSS);     % Assume o maximo da tabela
end

% Extrai um registro específico da tabela, correspondente a frequencia de operação definida pelo usuário
% Procura o valor da tabela que seja mais próximo do valor da frequencia de operação
 [v,pos]=min(abs(T.FreqBCSS-Freq));   % Extrai a posição (pos) da menor diferença entre a frequencia dada e um valor de frequencia da tabela
if v==0                          % Se a diferença minima é zero, o valor de frequencia é igual ao da tabela - não precisa interpolar
    T = T(pos,:);             % Extrai o registro com valores já definidos
else                               % Valor é próximo, mas não é igual ao da tabela
    if T.FreqBCSS(height(T))==T.FreqBCSS(pos) % É o último da tabela, não tem como interpolar
        T = T(height(T),:);             % Assume a última linha da tabela
    else      % Indica que existe valor intermediário que deve ser interpolado
        [v,pos]=max(T.FreqBCSS>Freq);           % Indica a primeira posição (pos) em que a FreqBCSS é maior que a frequencia de operação
        T=InterpolaLinear(Freq,T(pos-1,:),T(pos,:));             % Retorna valores interpolados entre o anterior e o posterior ao ponto
    end
end
 
% Usamos a própria estrutura da tabela para colocar a faixa dos limites L e H, que é a faixa definida
% como sendo os limites que serão dados ao controlador
T{2,:}= [  0  T{1,2:10}.*(1-FxPercent{2,2:10})];          % Considera todos os percentuais associados ao alarme L
T{3,:}= [  0  T{1,2:10}.*(1+FxPercent{3,2:10})];          % Considera todos os percentuais associados ao alarme H

% Se quiser mostrar dados na tela (fase de depuração)
mostra=0;
if mostra
    clc   % limpa área de trabalho
    % Prepara cabeçalho da tabela que será ilustrada na tela
    %fprintf('%s\t%s\t%s\t%s\t%s\n',"NOME DA VARIAVEL        ","Minimo","Máximo","Delta","Delta %")
    fprintf('%s\n',"     Freq       I.Total    I.Torque    P.Suc    P.Descarga    P.Diff    P.A.Natal   P.Chegada    P.Downhole   T.Chegada")
    fprintf('%s\n',"----------------------------------------------------------------------------------------------------------------------------------------------")   
    for i=1:3
        fprintf('%10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f   %10.2f      %10.2f    %10.2f \n',...
            T.FreqBCSS(i),T.TotalCurrent(i),T.TorqueCurrent(i),...
            T.IntakePressure(i),T.DischargePressure(i),T.DifferentialPressure(i),...
            T.XTreePressure(i),T.ProductionSurfacePressure(i),T.DownholePressure(i),...
            T.ProductionSurfaceTemperature(i));
    end
end

% =========================================================================================
% ===============================   FIM DO PROGRAMA PRINCIPAL =============================
% =========================================================================================
function T=InterpolaLinear(Freq,Tmin, Tmax);
T=Tmin;                        % Só para assumir estrutura da tabela
T.FreqBCSS=Freq;     % Assume o valor da frequencia definida (que não consta na tabela original)
T.TotalCurrent=linear(Freq,Tmin.FreqBCSS, Tmax.FreqBCSS,Tmin.TotalCurrent,Tmax.TotalCurrent);
T.TorqueCurrent=linear(Freq,Tmin.FreqBCSS, Tmax.FreqBCSS,Tmin.TorqueCurrent,Tmax.TorqueCurrent);
T.IntakePressure=linear(Freq,Tmin.FreqBCSS, Tmax.FreqBCSS,Tmin.IntakePressure,Tmax.IntakePressure);
T.DischargePressure=linear(Freq,Tmin.FreqBCSS, Tmax.FreqBCSS,Tmin.DischargePressure,Tmax.DischargePressure);
T.DifferentialPressure=linear(Freq,Tmin.FreqBCSS, Tmax.FreqBCSS,Tmin.DifferentialPressure,Tmax.DifferentialPressure);
T.XTreePressure=linear(Freq,Tmin.FreqBCSS, Tmax.FreqBCSS,Tmin.XTreePressure,Tmax.XTreePressure);
T.ProductionSurfacePressure=linear(Freq,Tmin.FreqBCSS, Tmax.FreqBCSS,Tmin.ProductionSurfacePressure,Tmax.ProductionSurfacePressure);
T.DownholePressure=linear(Freq,Tmin.FreqBCSS, Tmax.FreqBCSS,Tmin.DownholePressure,Tmax.DownholePressure);
T.ProductionSurfaceTemperature=linear(Freq,Tmin.FreqBCSS, Tmax.FreqBCSS,Tmin.ProductionSurfaceTemperature,Tmax.ProductionSurfaceTemperature);

% Só para visualizar na fase de depuração
% X=Tmin;
% X(2,:)=T;
% X(3,:)=Tmax;
% X

% ===========================================
function y=linear(x,x1,x2,y1,y2);    % Para calcular a interpolação linear propriamente dita
a=(y2-y1)/(x2-x1);                           % Coeficiente angular da reta
b = y1-a*x1;                                     % Coeficiente linear da reta
y=a*x+b;                                          % Calculo da função no ponto x

