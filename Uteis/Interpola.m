function New=Interpola(Freq,Press,T,Indice);
% Busca pontos da vizinhança para proceder a interpolação bilinear
Pontos=selPontos(Freq,Press,T);

% Avalia se os 4 pontos da vizinhança foram encontrados
if height(Pontos)==4                      % Os 4 pontos foram selecionados
    f1=Pontos.FreqBCSS(1);
    f2=Pontos.FreqBCSS(3);
    p1=Pontos.PressChegada(1);
    p2=Pontos.PressChegada(2);
    % Executa a interpolação bilinear para cada uma das variáveis
    VazaoOleo=Bilinear(Freq,Press,f1,f2,p1,p2,Pontos.VazaoOleo(1),Pontos.VazaoOleo(2),Pontos.VazaoOleo(3),Pontos.VazaoOleo(4));
    VazaoLiq=Bilinear(Freq,Press,f1,f2,p1,p2,Pontos.VazaoLiquido(1),Pontos.VazaoLiquido(2),Pontos.VazaoLiquido(3),Pontos.VazaoLiquido(4));
    Twh=Bilinear(Freq,Press,f1,f2,p1,p2,Pontos.Twh(1),Pontos.Twh(2),Pontos.Twh(3),Pontos.Twh(4));
    Pwh=Bilinear(Freq,Press,f1,f2,p1,p2,Pontos.Pwh(1),Pontos.Pwh(2),Pontos.Pwh(3),Pontos.Pwh(4));
    DeltaP=Bilinear(Freq,Press,f1,f2,p1,p2,Pontos.DeltaP(1),Pontos.DeltaP(2),Pontos.DeltaP(3),Pontos.DeltaP(4));
    Psuc=Bilinear(Freq,Press,f1,f2,p1,p2,Pontos.PressSuccao(1),Pontos.PressSuccao(2),Pontos.PressSuccao(3),Pontos.PressSuccao(4));

    New=Pontos(1,:); % Só para criar estrutura de tabela igual aos pontos coletados
    
    % Atualiza dados do registro com os valores interpolados
    New.FreqBCSS=Freq;
    New.PressChegada=Press;
    New.VazaoOleo=VazaoOleo;
    New.VazaoLiquido=VazaoLiq;
    New.Twh=Twh;
    New.Pwh=Pwh;
    New.DeltaP=DeltaP;
    New.PressSuccao=Psuc;
end

% Define se vai voltar tabela ou o valor especifico de um indice da tabla pré-estabelecido
if nargin==4                               % Passou indice
    New=table2array(New);       % Converte tabela em numérico
    New=New(Indice);                % Resgata indice específico da tabela
end    
 
%==================================================================
% ====================   FIM DO PROGRAMA PRINCIPAL  =================
%==================================================================
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


%==================================================================




