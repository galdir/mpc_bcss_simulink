function [QMin, QMax,PSucMin,PSucMax, PChegadaMin,PChegadaMax]=ProtecoesMapas(MatrizSimulador,BTP,Freq);
% Para calcular os limites dos mapas para uma determinada condição de frequência
%
% Em fase de depuração, permite plotar a linha vertical do mapa, para a
% frequencia especifica definida
%
% IMPORTANTE:
% Para não confindir os limites que são plotados no mapa, é conveniente que  o parâmetro
% gridP seja menor ou igual ao grid de pressão definido para gerar o mapa utilizado.
%

% Recebida a variável MatrizSimulador que corresponde a matriz do simulador Petrobras
% Coluna1 = FreqBCSS
% Coluna2=PressChegada
% Coluna3=VazaoOleo
% Coluna4=VazaoLiquido
% Coluna5=Twh
% Coluna6=Pwh
% Coluna7=DeltaP
% Coluna8=PressSuccao


%======================================================
% Limites para as interpolações 
PontoPChegada=unique(MatrizSimulador(:,2));   % Pontos que compõe o grid da PChegada
PMin=min(PontoPChegada);       % Assume o valor minimo de PChegada existente
PMax=max(PontoPChegada);    % Assume o valor máximo de PChegada existente

% ======================================================
% Cria uma tabela temporária para avaliar as várias condições de Pressão de
% Chegada em função da Frequencia definida
T=[];
 for i=1:height(PontoPChegada)    % Para cada valor de PChegada existente
       T(i,:)=Interpola(Freq,PontoPChegada(i),MatrizSimulador);    % Cria novo registro com base no valor interoplado
        i=i+1;    % Incrementa contador de registros
 end

% Avalia condição de Up e Downthrust
 [Qdt, Qut,Condicao,Cor]=AvaliaCondicao(T,BTP);
 
% Define valores iniciais para os limites de Vazão, PSuc e PChegada
QMin=9999999;
QMax=0;
PSucMin=9999999;
PSucMax=0;
PChegadaMin=9999999;
PChegadaMax=0;

 % Varre a tabela temporária para extrair os limites desejados
 for i=1:height(T)
     if Condicao(i)=="Normal"
         QMin=min(QMin,T(i,3));         
         QMax=max(QMax,T(i,3));         
         PSucMin=min(PSucMin,T(i,8));
         PSucMax=max(PSucMax,T(i,8));
         PChegadaMin=min(PChegadaMin,T(i,2));
         PChegadaMax=max(PChegadaMax,T(i,2));
     end
 end
 
% Habilite em fase de depuração se quiser ver a plotagem dos pontos verticais associados a frequencia selecionada
% PlotarCondicao(T,Cor,Freq);
 
% =============================================================================
% ========================    FIM DO PROGRAMA PRINCIPAL=========================
% =============================================================================
 function PlotarCondicao(T,Cor,Freq);
close all

Tam=0.1;       % Tamanho proporção do cículo na plotagem
% Mostra Mapa da Pressão de Chegada x Frequencia
figure(1)
set(gcf,'position', [280   500   410  340])
grid on
hold on
scatter(T(:,1),T(:,2),Tam*T(:,3),Cor,'filled')
title(strcat("Frequência = ",num2str(Freq),"Hz"));

% Este colormap associa verde, vermelho e amarelo quando há 3 condições e funciona no Mapa
% No caso, como é para uma frequencia específica apenas 2 condições são
% percebidas, cuidado para não confundir com as cores do mapa nem dá para
% associar verde como bom e vermelho como ruim
colormap(prism)

xlabel('Frequência [Hz]')
ylabel('Pressão de Chegada [Kgf/cm^2]')

% Mostra Mapa da Pressão de Sucção x Frequencia
figure(2)
set(gcf,'position', [700   500   410  340])
grid on
hold on
scatter(T(:,1),T(:,8),Tam*T(:,3),Cor,'filled')
title(strcat("Frequência = ",num2str(Freq),"Hz"));
colormap(prism)
xlabel('Frequência [Hz]')
ylabel('Pressão de Sucção [Kgf/cm^2]')

% Mostra Mapa da Vazão de óleo x Frequencia
figure(3)
set(gcf,'position', [280   70   410  340])
grid on
hold on
scatter(T(:,1),T(:,3),Tam*T(:,3),Cor,'filled')
title(strcat("Frequência = ",num2str(Freq),"Hz"));
colormap(prism)
xlabel('Frequência [Hz]')
ylabel('Vazão de Óleo [m^3/dia]')



