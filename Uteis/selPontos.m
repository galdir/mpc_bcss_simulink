function Pontos=selPontos(Freq,Press,T);
% Conhecido (Freq,Press), ponto no qual se quer calcular os dados,
% Seleciona os 4 pontos da tabela T para proceder a interpolação Bilinear

Pontos=T(1,:);        % Apenas para inicializar/criar a tabela. Valores serão atribuidos 
% ================================================================
% Confere os pontos que fazem parte do grid de simulação da Frequencia
F=[];        % Inicializa vetor que vai guardar os pontos de interpolação
gridF=unique(T.FreqBCSS)';     % Armezena grid utilizado na simulação da Frequencia

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
gridP=unique(T.PressChegada)';   % Armezena grid utilizado na simulação da Pressão de Chegada
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
%if ~isempty(F) & ~isempty(P) 
    Pontos(1,:) = T(T.FreqBCSS==F(1) & T.PressChegada ==P(1), :);    % Extrai o registro P11 da Tabela original   
    Pontos(2,:) = T(T.FreqBCSS==F(1) & T.PressChegada ==P(2), :);    % Extrai o registro P12 da Tabela original   
    Pontos(3,:) = T(T.FreqBCSS==F(2) & T.PressChegada ==P(1), :);    % Extrai o registro P21 da Tabela original   
    Pontos(4,:) = T(T.FreqBCSS==F(2) & T.PressChegada ==P(2), :);    % Extrai o registro P22 da Tabela original   
%end
