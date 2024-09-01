function [sys,x0]=sanima(t,x,u,flag,T,BTP,FIni,PChegaIni,PSucIni);
% Animação para atualização do ponto de operação nos mapas em tempo de simulação

global PontoMapa1
global PontoMapa2
%==================================================================================
% Inicialização
if flag==0,
  monta_mapas(T,BTP);                        % Apenas na inicialização, monta mapas como pano de fundo
  figure(1)
  subplot(1,2,1) 
  PontoMapa1=plot(FIni,PChegaIni,'kx');    % Inicializa variável que guarda o ponto de operação
  subplot(1,2,2) 
  PontoMapa2=plot(FIni,PSucIni,'kx');    % Inicializa variável que guarda o ponto de operação
  
  % Inicializa para S-Function
  ninput=4;                                 % Frequencia, Pressão de Chegada, Pressão de Sucção, Rastro do plot
  nout=0;                                    % Não tem saida, só o plot
  x0=[];                                        % Não armazena estados internos
  sys= [0;size(x0,1); nout; ninput;0;0];
%==================================================================================
% Contas para cada passo do simulink
elseif flag==2
% Frequencia=u(1);
% Pressao_Chegada=u(2);
% Pressão de Sucção = u(3);
    Rastro_Habilitado = u(4);
    if Rastro_Habilitado    % Se for para plotar o rastro, guarda posição atual
        F1=PontoMapa1.XData;    % Ponto de frequencia no mapa1 antes de atualizar
        P1=PontoMapa1.YData;    % Ponto de  pressão no mapa1 antes de atualizar
        F2=PontoMapa2.XData;    % Ponto de frequencia no mapa2 antes de atualizar
        P2=PontoMapa2.YData;    % Ponto de  pressão no mapa2 antes de atualizar
    end
    % Atualiza nova posição nos mapas
   figure(1)
   subplot(1,2,1)   % Atualiza mapa de pressão de chegada
    set(PontoMapa1, 'xdata', u(1), 'ydata', u(2));   % Desloca de acordo com as entradas
    subplot(1,2,2)  % Atualiza mapa da pressão de sucção
    set(PontoMapa2, 'xdata', u(1), 'ydata', u(3));   % Desloca de acordo com as entradas
    if Rastro_Habilitado    % Se for para plotar o rastro
        subplot(1,2,1)    % Atualiza mapa de pressão de chegada
        plot([ F1  u(1)],[P1  u(2)],'k')
        subplot(1,2,2)   % Atualiza mapa de pressão de chegada
        plot([ F2  u(1)],[P2  u(3)],'k')
    end
    drawnow;
    sys=[];
end
%==================================================================================
%==================================================================================
%==================================================================================
function monta_mapas(Tsim,BTP)
close all
figure(1)
set(gcf,'position',[230   245   1000   400]);
set(gcf,'MenuBar','none');
set(gcf,'name','Mapas de Operação');

[Qdt, Qut,Condicao,Cor]=AvaliaCondicao(Tsim,BTP);
gridFreq=unique(Tsim.FreqBCSS);   % Verifica o grid da frequencia
Tam=0.3*10/height(gridFreq);       % Usa o tamanho do grid para propor o tamanho do cículo na plotagem

% =================================================
% Monta as figuras para os mapas
subplot(1,2,1) % Mapa de Frequência x Pressão de Chegada
hold on
grid on
axis([ 39.9   60   10   50])
xlabel('Frequencia [Hz]');
ylabel('Pressão de Chegada [Kgf/cm^2]');
scatter(Tsim.FreqBCSS,Tsim.PressChegada,Tam*Tsim.VazaoOleo,Cor,'filled')
colormap(prism)

% =============
subplot(1,2,2)    % Mapa de Frequência x Pressão de Sucção
hold on
grid on
axis([ 39.9  60   65   105])
xlabel('Frequencia [Hz]');
ylabel('Pressão de Sucção [Kgf/cm^2]');
scatter(Tsim.FreqBCSS,Tsim.PressSuccao,Tam*Tsim.VazaoOleo,Cor,'filled')
colormap(prism)
%==================================================================================

