function [QMin, QMax,PSucMin,PSucMax, PChegadaMin,PChegadaMax]=ProtecoesMapas(Tt,BTP,Freq,gridP);
% Para calcular os limites dos mapas para uma determinada condição de frequência
%
% Em fase de depuração, permite plotar a linha vertical do mapa, para a
% frequencia especifica definida
%
% IMPORTANTE:
% Para não confindir os limites que são plotados no mapa, é conveniente que  o parâmetro
% gridP seja menor ou igual ao grid de pressão definidopara gerar o mapa utilizado.
%

%======================================================
% Limites para as interpolações 
PMin=min(Tt.PressChegada);       % Assume o valor minimo de PChegada existente na tabela do simulador
PMax=max(Tt.PressChegada);    % Assume o valor máximo de PChegada existente na tabela do simulador

% ======================================================
% Cria uma tabela temporária para avaliar as várias condições de Pressão de
% Chegada em função da Frequencia definida
T=Tt(1,:);    % Só para criar estrutura da tabela
i=1;   %   Contador
 for Pchegada=PMin:gridP:PMax
       T(i,:)=Interpola(Freq,Pchegada,Tt);    % Cria novo registro com base no valor interoplado
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
%          QMin=min(QMin,T.VazaoLiquido(i,1));         
%          QMax=max(QMax,T.VazaoLiquido(i,1));         
         QMin=min(QMin,T.VazaoOleo(i,1));         
         QMax=max(QMax,T.VazaoOleo(i,1));         
         PSucMin=min(PSucMin,T.PressSuccao(i,1));
         PSucMax=max(PSucMax,T.PressSuccao(i,1));
         PChegadaMin=min(PChegadaMin,T.PressChegada(i,1));
         PChegadaMax=max(PChegadaMax,T.PressChegada(i,1));
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
scatter(T.FreqBCSS,T.PressChegada,Tam*T.VazaoOleo,Cor,'filled')
title(strcat("Frequência = ",num2str(Freq),"Hz"));

% Este colormap associa verde, vermelho e ameralo quando há 3 condições e funciona no Mapa
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
scatter(T.FreqBCSS,T.PressSuccao,Tam*T.VazaoOleo,Cor,'filled')
title(strcat("Frequência = ",num2str(Freq),"Hz"));
colormap(prism)
xlabel('Frequência [Hz]')
ylabel('Pressão de Sucção [Kgf/cm^2]')

% Mostra Mapa da Pressão de chegada x  Vazão de óleo
figure(3)
set(gcf,'position', [280   70   410  340])
grid on
hold on
scatter(T.VazaoOleo,T.DeltaP,Tam*T.VazaoOleo,Cor,'filled')
title(strcat("Frequência = ",num2str(Freq),"Hz"));
colormap(prism)
xlabel('Vazão de Óleo [m^3/dia]')
ylabel('Delta P [Kgf/cm^2]')

% Mostra Mapa da Pressão de chegada x  Vazão de liquido
figure(4)
set(gcf,'position', [700   70   410  340])
grid on
hold on
scatter(T.VazaoLiquido,T.DeltaP,Tam*T.VazaoOleo,Cor,'filled')
title(strcat("Frequência = ",num2str(Freq),"Hz"));
colormap(prism)
xlabel('Vazão de Liquido [m^3/dia]')
ylabel('Delta P [Kgf/cm^2]')



