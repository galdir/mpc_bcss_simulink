%% Plots do caminho percorrido pela solução
% Mostra evolução da solução nas duas ações de controle
% 
% Usada para plotar resultado a partir do Exemplo 4 em diante
close all
figure(1)
set(gcf,'position',[120  400  560  420]);
subplot(2,1,1)
plot(u_cl(:,1),'bx')
ylabel('Velocidade linear (v)')
grid on
subplot(2,1,2)
plot(u_cl(:,2),'bx')
ylabel('Velocidade angular (omega)')
grid on

% Mostra evolução da solução na posição e orientação do robô
figure(2)
set(gcf,'position',[690  400  560  420]);
axis([ -3   3  -3  3])
hold on
grid on
Titulo=title(strcat("Passo 1 de ",num2str(width(xx1))));

A=0.1;             % Amplitude do vetor para o plot

% Plota o alvo desejado na especificação do problema
plot(xs(1), xs(2),'ro');                                                                        % Plota circulo no ponto
plot([xs(1)  xs(1)+A*cos(xs(3))],[xs(2)   xs(2)+A*sin(xs(3))],'r');   % Plota vetor com amplitude A e angulo Teta

% Se o exemplo usa obstáculo, plota circulo no local
if Obstaculo   
    Plota_Circulo(obs_x,obs_y,obs_r,1,'r')
end

% Tamaho do raio do robô default. Pode de ajustado para restrição dinâmica
RaioRobo=0.3;                                                 
        

% Prepara vetores de plots de k até o horizonte de predição = N
plotagens=plot_vetores(xx1(:,:,1),N,A,RaioRobo,Obstaculo);    % Inicializa vetores com dados do plot para a animação
for k=2:size(xx1,3)
    Titulo.String=strcat("Passo 1 de ",num2str(k));    
    pause(0.2)
    for i=1:N+1
        px=xx1(i,1,k);
        py=xx1(i,2,k);
        pTeta=xx1(i,3,k);
        % Atualiza animação dos pontos, orientações e circulos
        set(plotagens(i,1),'xdata',px,'ydata',py);
        set(plotagens(i,2),'xdata',[px   px+A*cos(pTeta)],'ydata',[py   py+A*sin(pTeta)]);
        if Obstaculo
            RaioRobo=RaioProtecaoDinamica(px,py);     % Tamanho do raio de proteção em função das coordenadas
        end
        [nx  ny]=Pontos_circulo(px,py,RaioRobo);
        set(plotagens(i,3),'xdata',nx,'ydata',ny);
    end
end
drawnow;

%% ==============================================
%% Fim da rotina principal
function  plotagens=plot_vetores(M,Hp,A,R,Obstaculo)
    % Observe que M é uma matriz bimensional onde:
    % - A linha 1 representa o valor dos estados no instante k
    % - A linha 2 representa o valor dos estados preditos para o instante k+1
    % - A linha 3 representa o valor dos estados preditos para o instante k+2
    % .....
    % - A última linha representa o valor dos estados preditos para o último horizonte Hp
    
    pontos=[];          % Inicializa pontos de posição do robô
    vetores=[];         % Inicializa vetores que indicam a orientação do robô
    circulos=[];         % Inicializa circulos que serão plotados ao redor do robô (usado para limite de proteção para não colidir com obstáculo)
    for i=1:Hp+ 1
        px=M(i,1);        % Extrai posição da coordenada x do robô
        py=M(i,2);        % Extrai posição da coordenada y do robô
        pTeta=M(i,3);  % Extrai angulo de orientação do robô
        if Obstaculo
            Raio=RaioProtecaoDinamica(px,py);    % Extrai o tamanho do circulo de proteção em função das coordenadas
        else
            Raio=R;     % Raio fixo preestabelecido
        end

         % Diferencia cor do plot da consição atual das condições preditas
        if i==1             
            cor='b';
        else
            cor='k';
        end
        
        % Plota posição (x,y) do robô, atual e em todo o horizonte de predição
        pto=plot(px,py,strcat(cor,'o'));
        pontos=[pontos;pto];

        % Plota orientação do robô, atual e em todo o horizonte de predição
        v=plot([px  px+A*cos(pTeta)],[py   py+A*sin(pTeta)],cor);
        vetores=[vetores;v];
        
        % Plota circulo em torno do robô, atual e em todo o horizonte de predição
        c=Plota_Circulo(px,py,Raio,0,strcat(cor,'--'));
        circulos=[circulos ; c];
    end
    plotagens=[pontos vetores  circulos];
end

