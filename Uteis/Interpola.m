function New=Interpola(Freq,Press,T,Indice);
    % Busca pontos da vizinhança para proceder a interpolação bilinear
    Pontos=selPontos(Freq,Press,T);

    % Avalia se os 4 pontos da vizinhança foram encontrados
    if height(Pontos)==4                      % Os 4 pontos foram selecionados
        f1=Pontos(1,1);
        f2=Pontos(3,1);
        p1=Pontos(1,2);
        p2=Pontos(2,2);

        % Executa a interpolação bilinear para cada uma das variáveis
        VazaoOleo=Bilinear(Freq,Press,f1,f2,p1,p2,Pontos(1,3), Pontos(2,3),Pontos(3,3),Pontos(4,3));
        VazaoLiq=Bilinear(Freq,Press,f1,f2,p1,p2,Pontos(1,4), Pontos(2,4),Pontos(3,4),Pontos(4,4));
        Twh=Bilinear(Freq,Press,f1,f2,p1,p2,Pontos(1,5), Pontos(2,5),Pontos(3,5),Pontos(4,5));
        Pwh=Bilinear(Freq,Press,f1,f2,p1,p2,Pontos(1,6), Pontos(2,6),Pontos(3,6),Pontos(4,6));
        DeltaP=Bilinear(Freq,Press,f1,f2,p1,p2,Pontos(1,7), Pontos(2,7),Pontos(3,7),Pontos(4,7));
        Psuc=Bilinear(Freq,Press,f1,f2,p1,p2,Pontos(1,8), Pontos(2,8),Pontos(3,8),Pontos(4,8));

        % Atualiza dados do registro com os valores interpolados
        New=[ Freq  Press  VazaoOleo  VazaoLiq   Twh   Pwh   DeltaP   Psuc];
    end

    % Define se vai voltar tabela ou apenas o valor especifico de um indice pré-estabelecido da tabela 
    if nargin==4                               % Passou indice
        New=New(Indice);                % Resgata indice específico da tabela
    end    
end
 %==================================================================
% ====================   FIM DO PROGRAMA PRINCIPAL  =================
%==================================================================
%% =================================================================
function Pontos=selPontos(Freq,Press,T);
    % Conhecido (Freq,Press), ponto no qual se quer calcular os dados,
    % Seleciona os 4 pontos da matriz T para proceder a interpolação Bilinear

    % ================================================================
    % Confere os pontos que fazem parte do grid de simulação da Frequencia
    F=[];        % Inicializa vetor que vai guardar os pontos de interpolação
    %gridF=unique(T.FreqBCSS)';     % Armazena grid utilizado na simulação da Frequencia
    gridF=unique(T(:,1))';                     % Armazena grid utilizado na simulação da Frequencia

    % Verifica se o valor de frequencia está dentro do grid para não dar erro de interpolação
    if Freq<min(gridF) |  Freq>max(gridF)     % Avalia se o ponto fornecido para interpolar está dentro da faixa
        disp(strcat("Ponto de Frequencia ",num2str(Freq,'%.1f'),"Hz está fora da faixa de interpolação [ ",num2str(min(gridF),'%.0f'),"  ",num2str(max(gridF),'%.0f'),"] Hz"))
        if Freq<min(gridF) Freq=min(gridF);   end      % Se valor passado está abaixo da faixa, assume o valor mínimo do grid definido     
        if Freq>max(gridF) Freq=max(gridF); end     % Se valor passado está acima da faixa, assume o valor máximo do grid definido
        disp(strcat("Cálculo feito com base na Frequencia = ",num2str(Freq,'%.0f'),"Hz"))
        beep
    end

    if sum(Freq==gridF)      % Ponto de Frequencia para interpolar já existe no grid da simulação, não precisa fazer contas
        F(1)=Freq;                  % Assume o valor da frequencia no ponto 1
        F(2)=Freq;                  % Assume o valor da frequencia no ponto 2
    else
        [val,pos]=max(gridF>Freq);   % Detecta ponto que a Frequencia no grid é maior do que a frequencia no ponto
        F(1)=gridF(pos-1);                 % Posição anterior ao máximo é o ponto em que a Frequencia no grid é menor do que a Frequencia no ponto
        F(2)=gridF(pos);                     % Posição do máximo é o ponto em que a Frequencia no grid é maior do que a Frequencia no ponto
    end

    % ================================================================
    % Confere os pontos que fazem parte do grid de simulação da Pressão de Chegada
    P=[];             % Inicializa vetor que vai guardar os pontos de interpolação
    % gridP=unique(T.PressChegada)';   % Armazena grid utilizado na simulação da Pressão de Chegada
    gridP=unique(T(:,2))';   % Armazena grid utilizado na simulação da Pressão de Chegada
    if Press<min(gridP) |  Press>max(gridP)     % Avalia se o ponto fornecido para interpolar está dentro da faixa
        disp(strcat("Ponto P.Chegada ",num2str(Press,'%.1f')," Kgf/cm2 está fora da faixa de interpolação [ ",num2str(min(gridP),'%.0f'),"   ",num2str(max(gridP),'%.0f')," ] Kgf/cm2"))
        if Press<min(gridP) Press=min(gridP);   end      % Se valor passado está abaixo da faixa, assume o valor mínimo do grid definido     
        if Press>max(gridP) Press=max(gridP); end     % Se valor passado está acima da faixa, assume o valor máximo do grid definido
        disp(strcat("Cálculo feito com base na Pressão de Chegada  = ",num2str(Press,'%.0f'),"Kgf/cm2"))
        beep
    end
    if sum(Press==gridP)                  % Ponto de Pressão para interpolar já existe no grid da simulação
        P(1)=Press;                              % Assume o valor da pressão no ponto 1
        P(2)=Press;                              % Assume o valor da pressão no ponto 2
    else
        [val,pos]=max(gridP>Press);  % Detecta ponto que a Pressão no grid é maior do que a Pressão no ponto
        P(1)=gridP(pos-1);                % Posição anterior ao máximo é o ponto em que a Pressão no grid é menor do que a Pressão no ponto
        P(2)=gridP(pos);                    % Posição do máximo é o ponto em que a Pressão no grid é maior do que a Pressão no ponto
    end 

    % ================================================================
    % Segue interpolação bilinear apenas se os pontos existirem e tiverem sido corretamente selecionados
        Pontos(1,:) = T(T(:,1)==F(1) & T(:,2)==P(1), :);    % Extrai o registro P11 da Tabela original
        Pontos(2,:) = T(T(:,1)==F(1) & T(:,2)==P(2), :);    % Extrai o registro P12 da Tabela original 
        Pontos(3,:) = T(T(:,1)==F(2) & T(:,2)==P(1), :);    % Extrai o registro P21 da Tabela original
        Pontos(4,:) = T(T(:,1)==F(2) & T(:,2)==P(2), :);     % Extrai o registro P22 da Tabela original
end
%% =============================================================================
function f=Bilinear(x,y,x1,x2,y1,y2,f11,f12,f21,f22);
    % Recebe as coordenadas dos pontos selecionados e procede a interpolação
    % da respectiva variável envolvida
    % è importante observar que se um dos pontos selecionados existe no grid da
    % simulação, não é necessário ser interpolado, pois a conta bilinear vai
    % gerar um NaN. Neste caso, fazemos apenas a interpolação linear da outra variável

    % Se os pontos de Frequencia e Pressão já existem no grid da simulação
    % Não precisa interpolar. Assume valor padrão da função
    f=f11;     % Qualquer um dos 4 pontos teriam o mesmo valor (f11=f12=f21=f22)

    % Calcula os valores de f em função da interpolação definida
    % Os pontos de Frequencia e Pressão não existem no grid da simulação
    % Interpolação BILINEAR
    if (x1~=x2) & (y1~=y2)     
        K=(x2-x1)*(y2-y1);     % Numerador que vai aparecer em todas as contas
        f=f11*(x2-x)*(y2-y) + f21*(x-x1)*(y2-y) + f12*(x2-x)*(y-y1) + f22*(x-x1)*(y-y1);
        f=f/K;
    end

    % Os pontos de Frequencia existem no grid da simulação
    % Interpolação LINEAR com base na Pressao
    if (x1==x2) & (y1~=y2)        % Frequencia escolhida existe no grid. Só precisa fazer a interpolação linear da Pressão
        a=(f12-f11)/(y2-y1);        % Coeficiente angular da reta
        b=f11-a*y1;                      % Coeficiente linear da reta
        f=a*y+b;                            % Valor interpolado no ponto
    end

    % Os pontos de Pressão existem no grid da simulação
    % Interpolação LINEAR com base na Frequencia
    if (x1~=x2) & (y1==y2)       % Pressão escolhida existe no grid. Só precisa fazer a interpolação linear da Frequencia
        a=(f22-f11)/(x2-x1);        % Coeficiente angular da reta
        b=f11-a*x1;                      % Coeficiente linear da reta
        f=a*x+b;                            % Valor interpolado no ponto
    end
end
%==================================================================




