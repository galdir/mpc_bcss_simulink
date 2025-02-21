function [sys,x0]=AnimaMapa(t,x,u,flag,PlotaMapas,MatrizSimulador,MatrizSimuladorContas,BTP,FreqIni,PMonIni,PSucIni,VazaoIni,FreqAlvoIni,PMonAlvoIni,HabilitaRastro,TabelaIsometricas,MargemPercentual,PMonAlvoMaxMin)
% Anima��o para atualiza��o do ponto de opera��o nos mapas em tempo de simula��o 

    global TituloMapa1                  % T�tulo no mapa de Frequ�ncia x PChegada
    global SubTituloMapa1            % SubT�tulo no mapa de Frequ�ncia x PChegada
    global TituloMapa2                   % T�tulo no mapa de Frequ�ncia x PSuc
    global SubTituloMapa2            % SubT�tulo no mapa de Frequ�ncia x PSuc
    global TituloMapa3                  % T�tulo no mapa de Frequ�ncia x Vazao de �leo
    global SubTituloMapa3            % SubT�tulo no mapa de Frequ�ncia x Vazao de �leo
    global IsoVazaoENG               % Plot da Isometrica de vaz�o correspondente ao alvo da engenharia

    global OperacaoMapa1          % Ponto de operacao no mapa de Frequ�ncia x PChegada
    global SetPointMapa1             % Ponto de set-point �timo no mapa de Frequ�ncia x PChegada
    global AlvoENGMapa1           % Ponto alvo da ENG no mapa de Frequ�ncia x PChegada

    global OperacaoMapa2          % Ponto no mapa de Frequ�ncia x PSuc 
    global SetPointMapa2             % Ponto de set-point �timo no mapa de Frequ�ncia x PSuc
    global AlvoENGMapa2           % Ponto alvo da ENG no mapa de Frequ�ncia x PSuc

    global OperacaoMapa3          % Ponto no mapa de Frequ�ncia x Vaz�o de �leo 
    global SetPointMapa3             % Ponto de set-point �timo no mapa de Frequ�ncia x Vaz�o de �leo
    global AlvoENGMapa3            % Ponto alvo no mapa de Frequ�ncia x Vaz�o de �leo

    %% ==================================================================================
    % Inicializa��o da S-Function
    if flag==0
        ninput=8;                                 % Inicializa��o da Frequencia passada para o processo
                                                        % Inicializa��o da PChegada, PSuc, Vazao  (dados de medi��es do processo)
                                                        % Inicializa��o da Freq. e PMon propostas pelo otimizador
                                                         % Inicializa��o da FreqAlvo, PMonAlvo  (dados da Engenharia)
                                                         
        nout=0;                                    % N�o tem saida, s� o plot
        x0=[];                                        % N�o armazena estados internos
        % Prepara plotagem dos mapas
        if PlotaMapas                                                    % Se for para plotar os mapas
            monta_mapas(MatrizSimulador,MatrizSimuladorContas,BTP,TabelaIsometricas,MargemPercentual,PMonAlvoMaxMin);     % Apenas na inicializa��o, monta mapas como pano de fundo
            % Corrige unidades de press�o da opera��o [bar]  para unidades de press�o nos  mapas [Kgf/cm2]
            PMonIni=PMonIni*1.019716;
            PSucIni=PSucIni*1.019716;
            PMonAlvoIni=PMonAlvoIni*1.019716;
            %=======================
            %  Mapa 1 = PChegada x Frequ�ncia
            subplot(3,1,1)       % Inicializa��o das vari�veis que v�o compor o mapa
            TituloMapa1=title("Mapa PChegada x Frequ�ncia");
            SubTituloMapa1=subtitle("");
            SubTituloMapa1.Color='blue';
            OperacaoMapa1=plot(FreqIni,PMonIni,'ko');                   % Inicializa ponto de opera��o neste gr�fico
            SetPointMapa1=plot(FreqIni,PMonIni,'k*');                       % Inicializa no mesmo ponto de inicializa��o da opera��o
            AlvoENGMapa1=plot(FreqAlvoIni,PMonAlvoIni,'bx');       % Incializa alvo da ENG no mapa
            
            % Prepara isom�trica da vaz�o para o alvo da engenharia
            VazaoAlvoIni=Interpola(FreqAlvoIni, PMonAlvoIni,MatrizSimulador,3);  % Estima vaz�o em fun��o da Freq e Pressao
            
            VarX=TabelaIsometricas.FreqBCSS;                           % Vari�vel que vai compor o eixo X do Mapa   
            VarY=TabelaIsometricas.PressChegada;                    % Vari�vel que vai compor o eixo Y do Mapa
            VarProcurada=TabelaIsometricas.VazaoOleo;           % Vari�vel que vai compor a curva isom�trica no Mapa
%            [VarX,VarY]=IsoCurva(VarX,VarY,VarProcurada,VazaoAlvoIni,VazaoAlvoIni/300,1);    % Tabela, Vari�vel X, Vari�vel Y, Vari�vel procurada, Valor procurado, Toler�ncia, Escala
            [VarX,VarY]=IsoCurva(VarX,VarY,VarProcurada,VazaoAlvoIni,0.5,1);    % Tabela, Vari�vel X, Vari�vel Y, Vari�vel procurada, Valor procurado, Toler�ncia, Escala
            IsoVazaoENG=plot(VarX,VarY,'k--');                               % Guarda vari�vel para anima��o qdo alterar o alvo da ENG
            
            %=======================
            %  Mapa 2 = PSuc x Frequ�ncia
            subplot(3,1,2)    % Inicializa��o das vari�veis que v�o compor o mapa
            TituloMapa2=title("Mapa PSuc x Frequ�ncia");
            SubTituloMapa2=subtitle("");
            SubTituloMapa2.Color='blue';
            OperacaoMapa2=plot(FreqIni,PSucIni,'ko');                    % Inicializa ponto de opera��o neste gr�fico
            PSucOtimaIni=Interpola(FreqIni, PMonIni,MatrizSimulador,8);  % Estima em fun��o da Freq e Pressao
            SetPointMapa2=plot(FreqIni,PSucOtimaIni,'k*');              % Inicializa no mesmo ponto de inicializa��o da opera��o
            PSucAlvoEng=Interpola(FreqAlvoIni, PMonAlvoIni,MatrizSimulador,8);  % Estima em fun��o da Freq e Pressao
            AlvoENGMapa2=plot(FreqAlvoIni,PSucAlvoEng,'bx');     % Incializa alvo da ENG no mapa

            %=======================
            %  Mapa 3 = Vaz�o x Frequ�ncia
            subplot(3,1,3)    % Inicializa��o das vari�veis que v�o compor o mapa 3
            TituloMapa3=title("Mapa Vaz�o de �leo x Frequ�ncia");
            SubTituloMapa3=subtitle("");
            SubTituloMapa3.Color='blue';
            VazaoAlvoIni=Interpola(FreqAlvoIni, PMonAlvoIni,MatrizSimulador,3);  % Estima vaz�o em fun��o da Freq e Pressao
            OperacaoMapa3=plot(FreqIni,VazaoIni,'ko');                  % Inicializa ponto de opera��o neste gr�fico
            SetPointMapa3=plot(FreqIni,VazaoIni,'k*');               % Inicializa no mesmo ponto de inicializa��o da opera��o
            AlvoENGMapa3=plot(FreqAlvoIni,VazaoAlvoIni,'bx');      % Incializa alvo da ENG no mapa
        end
        sys= [0;size(x0,1); nout; ninput;0;0];
    %% ==================================================================================
    % Contas para cada passo do simulink
    elseif flag==2
        if PlotaMapas                                        % Se for para plotar os mapas. Caso negativo, n�o perde tempo !!
            % Resgata entradas do bloco para poder atualizar todos os mapas
            % Dados da Opera��o
            FreqOperacao=u(1);                        % Frequencia de opera��o [Hz] 
            PChegada=u(2)*1.019716;             % Press�o de Chegada convertida de bar para Kgf/cm2
            PSuc=u(3)*1.019716;                      % Press�o de Suc��o convertida de bar para Kgf/cm2
            VazaoOleo=u(4);                              % Vaz�o em m3/d
            
            % Dados do otimizador
            FreqOtima=u(5);                                % Frequencia proposta pelo Controlador
            PChegadaOtima=u(6)*1.019716;    % Press�o de Chegada (PMonAlvo) dada pelo otimizador  convertida de bar para Kgf/cm2
            PSucOtima=Interpola(FreqOtima, PChegadaOtima,MatrizSimulador,8);            % Estima em fun��o da Freq e Pressao
            VazaoOleoOtima=Interpola(FreqOtima, PChegadaOtima,MatrizSimulador,3);   % Estima em fun��o da Freq e Pressao

            % Dados desejados pela engenharia (Alvos)
            FreqAlvoENG=u(7);                                  % Frequ�ncia Alvo ENG atual
            PMonAlvoENG=u(8)*1.019716;              % PMon Alvo ENG atual, convertida de bar para Kgf/cm2
            PSucAlvoENG=Interpola(FreqAlvoENG, PMonAlvoENG,MatrizSimulador,8);    % Estima em fun��o da Freq e Pressao
            VazaoAlvoENG=Interpola(FreqAlvoENG, PMonAlvoENG,MatrizSimulador,3);  % Estima em fun��o da Freq e Pressao

            % Monta vetores sempre na sequencia de [Medido,   AlvoENG,  Otima]
            PChegada= [PChegada,        PMonAlvoENG,    PChegadaOtima];
            PSuc=         [PSuc,                 PSucAlvoENG,      PSucOtima];
            Vazao=       [VazaoOleo,        VazaoAlvoENG,    VazaoOleoOtima];
            Freq=          [FreqOperacao,  FreqAlvoENG,       FreqOtima];

            % Atualiza pontos nos 3 mapas de opera��o
            AtualizaMapa(1,OperacaoMapa1,AlvoENGMapa1,SetPointMapa1,HabilitaRastro,Freq,PChegada);
            AtualizaMapa(2,OperacaoMapa2,AlvoENGMapa2,SetPointMapa2,HabilitaRastro,Freq,PSuc);
            AtualizaMapa(3,OperacaoMapa3,AlvoENGMapa3,SetPointMapa3,HabilitaRastro,Freq,Vazao);
            
            % Atualiza valores nos t�tulos dos gr�ficos
            TituloMapa1.String=strcat("PChegada Medida=",num2str(PChegada(1),'%.1f'),"    (*) SetPoint=",num2str(PChegada(3),'%.1f'),"     Erro = ",num2str(PChegada(1)-PChegada(3),'%.1f')," [Kgm/cm2]");
            SubTituloMapa1.String=strcat("                                          (x) Alvo ENG=",num2str(PChegada(2),'%.1f'),"    Erro =",num2str(PChegada(1)-PChegada(2),'%.1f')," [Kgm/cm2]");

            TituloMapa2.String=strcat("PSuc Medida=",num2str(PSuc(1),'%.1f'),"    (*) SetPoint= ",num2str(PSuc(3),'%.1f'),"     Erro =",num2str(PSuc(1)-PSuc(3),'%.1f')," [Kgm/cm2]");
            SubTituloMapa2.String=strcat("                                    (x) Alvo ENG=",num2str(PSuc(2),'%.1f'),"     Erro =",num2str(PSuc(1)-PSuc(2),'%.1f')," [Kgm/cm2]");

            TituloMapa3.String=strcat("Vaz�o Estimada= ",num2str(Vazao(1),'%.2f'),"    (*) SetPoint=",num2str(Vazao(3),'%.2f'),"     Erro =",num2str(Vazao(1)-Vazao(3),'%.2f')," [m3/dia]");
            SubTituloMapa3.String=strcat("                                    (x) Alvo ENG=",num2str(Vazao(2),'%.2f'),"     Erro = ",num2str(Vazao(1)-Vazao(2),'%.2f')," [m3/dia]");
            
            % Atualiza curva de isom�trica da vaz�o correspondente ao alvo da ENG que pode ter alterado
            VarX=TabelaIsometricas.FreqBCSS;                           % Vari�vel que vai compor o eixo X do Mapa   
            VarY=TabelaIsometricas.PressChegada;                    % Vari�vel que vai compor o eixo Y do Mapa
            VarProcurada=TabelaIsometricas.VazaoOleo;           % Vari�vel que vai compor a curva isom�trica no Mapa
            [VarX,VarY]=IsoCurva(VarX,VarY,VarProcurada,VazaoAlvoENG,0.5,1);    % Tabela, Vari�vel X, Vari�vel Y, Vari�vel procurada, Valor procurado, Toler�ncia, Escala
            set(IsoVazaoENG,'xdata',VarX,'ydata',VarY)                % Atualiza no mapa a curva isom�trica da vaz�o 
            
           drawnow;
        end
        sys=[];
    end
end
%==================================================================================
%% ==================================================================================
function  AtualizaMapa(Mapa,OperacaoMapa,AlvoENGMapa,SetPointMapa,Rastro,Freq,Variavel)
  % Freq sempre na sequencia: [ Medido,   AlvoENG]
  % Vari�vel PChegada, PSuc e Vazao, sempre na sequencia: [ Medido,   AlvoENG,  Otima]
    subplot(3,1,Mapa)   % Resgata mapa que ser� atualizado
    
    if Rastro
        % Resgata valores anteriores para poder plotar rastro
        FreqOperacaoAntes=OperacaoMapa.XData;    % Ponto da frequencia de Opera��o antes de atualizar
        FreqAlvoAntes=AlvoENGMapa.XData;                % Ponto da frequencia Alvo da ENG antes de atualizar
        Variavel1Antes=OperacaoMapa.YData;              % Ponto da Vari�vel medida antes de atualizar
        Variavel2Antes=AlvoENGMapa.YData;                % Ponto da Vari�vel ALVO da ENG antes de atualizar
        Variavel3Antes=SetPointMapa.YData;                 % Ponto da Vari�vel �tima (dada pelo otimizador) antes de atualizar
        
        % Plota os rastros
        plot([ FreqOperacaoAntes  Freq(1)],[Variavel1Antes  Variavel(1)],'k')       % Rastro do ponto de opera��o
%         plot([ FreqAlvoAntes  Freq(2)],[Variavel2Antes  Variavel(2)],'b')            % Rastro do alvo desejado pela ENG
%         plot([ FreqAlvoAntes  Freq(3)],[Variavel3Antes  Variavel(3)],'k')             % Rastro do set point dado pelo otimizador
    end

    % Atualiza pontos no respetivo mapa
    set(OperacaoMapa, 'xdata',Freq(1), 'ydata', Variavel(1));    % Atualiza ponto de opera��o
    set(SetPointMapa, 'xdata', Freq(3), 'ydata', Variavel(3));      % Atualiza Set Point dado pelo otimizador
%     if Mapa~=2       % Se n�o quiser plotar ENG no Mapa de PSuc
        set(AlvoENGMapa, 'xdata', Freq(2), 'ydata', Variavel(2));     % Atualiza alvo desejado pela engenharia
%     end
%     ax=gca;
%     ax.Subtitle.HorizontalAlignment='right';
end

%% ==================================================================================
function monta_mapas(MatrizSimulador,MatrizSimuladorContas,BTP,TabelaIsometricas,MargemPercentual,PMonAlvoMaxMin)
    close all
    figure(1)
    set(gcf,'position',[10   5   570   875]);
    % set(gcf,'MenuBar','none');
    set(gcf,'name','MAPAS DE OPERA��O');

    [Qdt, Qut,Condicao,Cor]=AvaliaCondicao(MatrizSimulador,BTP);
    gridFreq=unique(MatrizSimulador(:,1));   % Verifica o grid da frequencia
    Tam=0.1*10/height(gridFreq);       % Usa o tamanho do grid para propor o tamanho do c�culo na plotagem

    % Varre grid definido para a frequ�ncia e calcula os limites de prote��o
    for i=1:height(gridFreq)
         [QMin(i), QMax(i),PSucMin(i),PSucMax(i), PChegadaMin(i),PChegadaMax(i)]=ProtecoesMapas(MatrizSimuladorContas,BTP,gridFreq(i));
    end

    % =================================================
    % Monta as figuras para os mapas
    subplot(3,1,1) % Mapa de Press�o de Chegada x Frequ�ncia 
    hold on
    grid on
    % axis([ 39.9   60   10   50])
    % xlabel('Frequencia [Hz]');
    ylabel('Press�o de Chegada [Kgf/cm^2]');
    scatter(MatrizSimulador(:,1),MatrizSimulador(:,2),Tam*MatrizSimulador(:,3),Cor,'filled')
    colormap(prism)
    % Tra�a limites do mapa
    plot(gridFreq,PChegadaMin,'r')
    plot(gridFreq,PChegadaMin*(1+MargemPercentual/100),'r--')
    plot(gridFreq,PChegadaMax,'b')
    plot(gridFreq,PChegadaMax*(1-MargemPercentual/100),'b--')
   % Tra�a limites PChegada Min e Max definidos pelo usu�rio
    plot([40 60],[PMonAlvoMaxMin(1)  PMonAlvoMaxMin(1)],'b--') 
    plot([40 60],[PMonAlvoMaxMin(2)  PMonAlvoMaxMin(2)],'r--') 
     
    eixo=axis;
    eixo(1)=39.9;
    axis(eixo);         % Drible para efeito visual na escala do plot 40Hz
    
    
    % Isocurvas do mapa 1
    %  Completa mapa com curvas de isovaz�o
    for i=250:50:500
        VarX=TabelaIsometricas.FreqBCSS;                           % Vari�vel que vai compor o eixo X do Mapa   
        VarY=TabelaIsometricas.PressChegada;                    % Vari�vel que vai compor o eixo Y do Mapa
        VarProcurada=TabelaIsometricas.VazaoOleo;           % Vari�vel que vai compor a curva isom�trica no Mapa
        [VarX,VarY]=IsoCurva(VarX,VarY,VarProcurada,i,0.5,1);    % Tabela, Vari�vel X, Vari�vel Y, Vari�vel procurada, Valor procurado, Toler�ncia, Escala
        plot(VarX,VarY,'k:')
        text(max(VarX),max(VarY)-1,strcat(num2str(i),"m^3/d"),'FontSize',8)
    end

    % =============
    subplot(3,1,2)    % Mapa de Press�o de Suc��o x Frequ�ncia
    hold on
    grid on
    % axis([ 39.9  60   65   105])
    % xlabel('Frequencia [Hz]');
    ylabel('Press�o de Suc��o [Kgf/cm^2]');
    scatter(MatrizSimulador(:,1),MatrizSimulador(:,8),Tam*MatrizSimulador(:,3),Cor,'filled')
    colormap(prism)
    % Tra�a limites do mapa
    plot(gridFreq,PSucMin,'r')
    plot(gridFreq,PSucMin*(1+MargemPercentual/100),'r--')
    plot(gridFreq,PSucMax,'b')
    plot(gridFreq,PSucMax*(1-MargemPercentual/100),'b--')
    eixo=axis;
    eixo(1)=39.9;
    axis(eixo);         % Drible para efeito visual na escala do plot 40Hz

   % Completa mapa 2 com curvas isob�ricas da PChegada
    for i=20:10:50
        VarX=TabelaIsometricas.FreqBCSS;                          % Vari�vel que vai compor o eixo X do Mapa   
        VarY=TabelaIsometricas.PressSuccao;                      % Vari�vel que vai compor o eixo Y do Mapa
        VarProcurada=TabelaIsometricas.PressChegada;    % Vari�vel que vai compor a curva isom�trica no Mapa
        [VarX,VarY]=IsoCurva(VarX,VarY,VarProcurada,i,0.5,1);    % Tabela, Vari�vel X, Vari�vel Y, Vari�vel procurada, Valor procurado, Toler�ncia, Escala
        plot(VarX,VarY,'k:')
        text(min(VarX),max(VarY)-1,strcat(num2str(i)),'FontSize',8)
    end

    % =============
    subplot(3,1,3)    % Mapa de Vaz�o x Frequ�ncia
    hold on
    grid on
    % axis([ 39.9  60   65   105])
    xlabel('Frequencia [Hz]');
    ylabel('Vaz�o de �leo [m3/dia]');
    scatter(MatrizSimulador(:,1),MatrizSimulador(:,3),Tam*MatrizSimulador(:,3),Cor,'filled')
    colormap(prism)
    % Tra�a limites do mapa
    plot(gridFreq,QMin,'b')
    plot(gridFreq,QMin*(1+MargemPercentual/100),'b--')
    plot(gridFreq,QMax,'r')
    plot(gridFreq,QMax*(1-MargemPercentual/100),'r--')
    eixo=axis;
    eixo(1)=39.9;
    axis(eixo);         % Drible para efeito visual na escala do plot 40Hz
    
   % Completa mapa 3 com curvas isob�ricas da PChegada
    for i=20:10:50
        VarX=TabelaIsometricas.FreqBCSS;                          % Vari�vel que vai compor o eixo X do Mapa   
        VarY=TabelaIsometricas.VazaoOleo;                         % Vari�vel que vai compor o eixo Y do Mapa
        VarProcurada=TabelaIsometricas.PressChegada;    % Vari�vel que vai compor a curva isom�trica no Mapa
        [VarX,VarY]=IsoCurva(VarX,VarY,VarProcurada,i,0.5,1);    % Tabela, Vari�vel X, Vari�vel Y, Vari�vel procurada, Valor procurado, Toler�ncia, Escala
        plot(VarX,VarY,'k:')
        text(min(VarX),min(VarY)-1,strcat(num2str(i)),'FontSize',8)
    end

end
%==================================================================================
