function Limites=ProtecaoDinamica(Freq,DP,FxPercent);
    % Função para calculo dos limites de alarmes de algumas variáveis. Estes não são valores
    % fixos, mas sim em função da frequência de operação. Assim, esta função recebe a
    % frequencia de operação e calcula os limites de proteção de algumas variáveis.
    %
    % Para isso, usaremos as tabelas fornecidas pela Petrobras como exemplo de Campo dos Goytacazes
    % Uma vez mantido o formato, as tabelas XLSX pode ser substituida por novas tabelas (novas referências)

    % A estrutura das tabelas de Protecao Dinâmica (DP) e respectivas Faixas Percentuais (DP-Faixas) está na forma:
    % Coluna1 = FreqBCSS
    % Coluna2 = TotalCurrent
    % Coluna3 = TorqueCurrent
    % Coluna4 = IntakePressure   (PSuc)
    % Coluna5 = DischargePressure (PDescarga)
    % Coluna6 = DifferentialPressure  (PDiff)
    % Coluna7 = XTreePressure (Pressão medida pelo TPT)  - NAO USAMOS
    % Coluna8 = ProductionSurfacePressure (PChegada)
    % Coluna9 = DownholePressure  (Pressão medida pelo PDG) - NAO USAMOS
    % Coluna10 = ProductionSurfaceTemperature ( Temperatura de Chegada)

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
    % Há outros critérios de proteção, a exemplo da variação brusca da
    % corrente. Casos como estes, porém, não são percebidos pelo modelo mas já
    % fazem parte das rotinas de proteção implementadas na empresa

    if nargin<1      % Não definiu argumento (só na fase de depuração)
        Freq=40.4;
    end
    if nargin<3       % Não passou as duas tabelas como parâmetro
        % Carrega as tabelas de referências para as proteções dinâmicas
        DP=readtable('DP.xlsx');                              % Tabela de valores de referência para as variáveis
        DP=table2array(DP);
        FxPercent=readtable('DP-Faixas.xlsx');     % Tabela das faixas percentuais para disparar os alarmes LL, L, H e HH
        fx=FxPercent(:,2:end);                                % Extrai a primeira coluna com os simbolos dos alarmes
        fx=table2array(fx);                                        % Converte a tabela em array
        FxPercent=[ [0;0;0;0] fx  ];                            % Preenche a primeira coluna com zeros para manter o mesmo indice da tabela DP
    end

    % Há algumas proteções, pois, o valor da frequencia de operação não
    % necessariamente está na tabela. Pode estar, inclusive, fora faixa coberta pela tabela
    if Freq<min(DP(:,1))       % Se o valor de frequencia for menor que o mínimo da tabela
        Freq=min(DP(:,1));     % Assume a frequência mínima da tabela
    end
    if Freq>max(DP(:,1))       % Se o valor de frequencia for maior que o maximo da tabela
        Freq=max(DP(:,1));     % Assume a frequência maxima da tabela
    end

    % Extrai um registro específico da tabela, correspondente a frequencia de operação definida pelo usuário
    % Procura o valor da tabela que seja mais próximo do valor da frequencia de operação
     [v,pos]=min(abs(DP(:,1)-Freq));   % Extrai a posição (pos) da menor diferença entre a frequencia dada e um valor de frequencia da tabela
    if v==0                          % Se a diferença minima é zero, o valor de frequencia é igual ao da tabela - não precisa interpolar
        DP = DP(pos,:);       % Extrai o registro (todas as colunas) com valores já definidos
    else                               % Valor é próximo, mas não é igual ao da tabela
        if DP(end,1)==DP(pos,1) % É o último da tabela, não tem como interpolar
            DP= DP(end,:);              % Assume a última linha da tabela
        else      % Indica que existe valor intermediário que deve ser interpolado
            [v,pos]=max(DP(:,1)>Freq);           % Indica a primeira posição (pos) em que a FreqBCSS é maior que a frequencia de operação
            DP=InterpolaLinear(Freq,DP(pos-1,:),DP(pos,:));             % Retorna valores interpolados entre o anterior e o posterior ao ponto
        end
    end

    % Usamos a própria estrutura da tabela para colocar a faixa dos limites L e H, que é a faixa definida
    % como sendo os limites que serão dados ao controlador
    Limites(1,:)= [ Freq  DP(1,2:end).*(1+FxPercent(3,2:end))];          % Considera todos os percentuais associados ao alarme H
    Limites(2,:)= [ Freq  DP(1,2:end).*(1-FxPercent(2,2:end))];           % Considera todos os percentuais associados ao alarme L

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
                T(i,1),T(i,2),T(i,3),T(i,4),T(i,5), T(i,6), T(i,7), T(i,8), T(i,9), T(i,10))
        end
    end
end

% =========================================================================================
% ===============================   FIM DO PROGRAMA PRINCIPAL =============================
% =========================================================================================
function T=InterpolaLinear(Freq,Tmin, Tmax);
    T=[];
    T(1)= Freq;   % Assume o valor da frequencia definida (que não consta na tabela original)
    for c=2:10;   % Varre as demais colunas 
        T(c)=  linear(Freq,Tmin(1), Tmax(1),Tmin(c),Tmax(c));   % Procede interpolação com base na Frequencia min/max e no Min/Mac de respectiva variável
    end
end
% ===========================================
function y=linear(x,x1,x2,y1,y2);    % Para calcular a interpolação linear propriamente dita
    a=(y2-y1)/(x2-x1);                           % Coeficiente angular da reta
    b = y1-a*x1;                                     % Coeficiente linear da reta
    y=a*x+b;                                          % Calculo da função no ponto x
end
