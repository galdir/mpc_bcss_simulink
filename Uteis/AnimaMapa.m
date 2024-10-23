function [sys,x0]=AnimaMapa(t,x,u,flag,PlotaMapas,MatrizSimulador,MatrizSimuladorContas,BTP,FreqIni,PMonIni,PSucIni,VazaoIni,FreqAlvoIni,PMonAlvoIni,HabilitaRastro,TabelaIsometricas,MargemPercentual)
% Animação para atualização do ponto de operação nos mapas em tempo de simulação 

    global TituloMapa1                  % Título no mapa de Frequência x PChegada
    global SubTituloMapa1            % SubTítulo no mapa de Frequência x PChegada
    global TituloMapa2                   % Título no mapa de Frequência x PSuc
    global SubTituloMapa2            % SubTítulo no mapa de Frequência x PSuc
    global TituloMapa3                  % Título no mapa de Frequência x Vazao de òleo
    global SubTituloMapa3            % SubTítulo no mapa de Frequência x Vazao de òleo
    global IsoVazaoENG               % Plot da Isometrica de vazão correspondente ao alvo da engenharia

    global OperacaoMapa1          % Ponto de operacao no mapa de Frequência x PChegada
    global SetPointMapa1             % Ponto de set-point ótimo no mapa de Frequência x PChegada
    global AlvoENGMapa1           % Ponto alvo da ENG no mapa de Frequência x PChegada

    global OperacaoMapa2          % Ponto no mapa de Frequência x PSuc 
    global SetPointMapa2             % Ponto de set-point ótimo no mapa de Frequência x PSuc
    global AlvoENGMapa2           % Ponto alvo da ENG no mapa de Frequência x PSuc

    global OperacaoMapa3          % Ponto no mapa de Frequência x Vazão de òleo 
    global SetPointMapa3             % Ponto de set-point ótimo no mapa de Frequência x Vazão de Óleo
    global AlvoENGMapa3            % Ponto alvo no mapa de Frequência x Vazão de òleo

    %% ==================================================================================
    % Inicialização da S-Function
    if flag==0
        ninput=10;                               % Inicialização da Frequencia passada para o processo
                                                        % Inicialização da PChegada, PSuc, Vazao  (dados de medições do processo)
                                                        % Inicialização da Frequencia proposta pelo otimizador
                                                        % Inicialização da PChegada, PSuc, Vazao (propostos pelo otimizador)
                                                        % Inicialização da FreqAlvo, PMonAlvo  (dados da Engenharia)
                                                         
        nout=0;                                    % Não tem saida, só o plot
        x0=[];                                        % Não armazena estados internos
        % Prepara plotagem dos mapas
        if PlotaMapas                                                    % Se for para plotar os mapas
            monta_mapas(MatrizSimulador,MatrizSimuladorContas,BTP,TabelaIsometricas,MargemPercentual);     % Apenas na inicialização, monta mapas como pano de fundo
            % Corrige unidades de pressão da operação [bar]  para unidades de pressão nos  mapas [Kgf/cm2]
            PMonIni=PMonIni*1.019716;
            PSucIni=PSucIni*1.019716;
            PMonAlvoIni=PMonAlvoIni*1.019716;
            %=======================
            %  Mapa 1 = PChegada x Frequência
            subplot(3,1,1)       % Inicialização das variáveis que vão compor o mapa
            TituloMapa1=title("Mapa PChegada x Frequência");
            SubTituloMapa1=subtitle("");
            SubTituloMapa1.Color='blue';
            OperacaoMapa1=plot(FreqIni,PMonIni,'ko');                   % Inicializa ponto de operação neste gráfico
            SetPointMapa1=plot(FreqIni,PMonIni,'k*');                       % Inicializa no mesmo ponto de inicialização da operação
            AlvoENGMapa1=plot(FreqAlvoIni,PMonAlvoIni,'bx');       % Incializa alvo da ENG no mapa
            
            % Prepara isométrica da vazão para o alvo da engenharia
            VazaoAlvoIni=Interpola(FreqAlvoIni, PMonAlvoIni,MatrizSimulador,3);  % Estima vazão em função da Freq e Pressao
            
            VarX=TabelaIsometricas.FreqBCSS;                           % Variável que vai compor o eixo X do Mapa   
            VarY=TabelaIsometricas.PressChegada;                    % Variável que vai compor o eixo Y do Mapa
            VarProcurada=TabelaIsometricas.VazaoOleo;           % Variável que vai compor a curva isométrica no Mapa
%            [VarX,VarY]=IsoCurva(VarX,VarY,VarProcurada,VazaoAlvoIni,VazaoAlvoIni/300,1);    % Tabela, Variável X, Variável Y, Variável procurada, Valor procurado, Tolerância, Escala
            [VarX,VarY]=IsoCurva(VarX,VarY,VarProcurada,VazaoAlvoIni,0.5,1);    % Tabela, Variável X, Variável Y, Variável procurada, Valor procurado, Tolerância, Escala
            IsoVazaoENG=plot(VarX,VarY,'k--');                               % Guarda variável para animação qdo alterar o alvo da ENG
            
            %=======================
            %  Mapa 2 = PSuc x Frequência
            subplot(3,1,2)    % Inicialização das variáveis que vão compor o mapa
            TituloMapa2=title("Mapa PSuc x Frequência");
            SubTituloMapa2=subtitle("");
            SubTituloMapa2.Color='blue';
            OperacaoMapa2=plot(FreqIni,PSucIni,'ko');                    % Inicializa ponto de operação neste gráfico
            PSucOtimaIni=Interpola(FreqIni, PMonIni,MatrizSimulador,8);  % Estima em função da Freq e Pressao
            SetPointMapa2=plot(FreqIni,PSucOtimaIni,'k*');              % Inicializa no mesmo ponto de inicialização da operação
            PSucAlvoEng=Interpola(FreqAlvoIni, PMonAlvoIni,MatrizSimulador,8);  % Estima em função da Freq e Pressao
            AlvoENGMapa2=plot(FreqAlvoIni,PSucAlvoEng,'bx');     % Incializa alvo da ENG no mapa

            %=======================
            %  Mapa 3 = Vazão x Frequência
            subplot(3,1,3)    % Inicialização das variáveis que vão compor o mapa 3
            TituloMapa3=title("Mapa Vazão de Óleo x Frequência");
            SubTituloMapa3=subtitle("");
            SubTituloMapa3.Color='blue';
            VazaoAlvoIni=Interpola(FreqAlvoIni, PMonAlvoIni,MatrizSimulador,3);  % Estima vazão em função da Freq e Pressao
            OperacaoMapa3=plot(FreqIni,VazaoIni,'ko');                  % Inicializa ponto de operação neste gráfico
            SetPointMapa3=plot(FreqIni,VazaoIni,'k*');               % Inicializa no mesmo ponto de inicialização da operação
            AlvoENGMapa3=plot(FreqAlvoIni,VazaoAlvoIni,'bx');      % Incializa alvo da ENG no mapa
        end
        sys= [0;size(x0,1); nout; ninput;0;0];
    %% ==================================================================================
    % Contas para cada passo do simulink
    elseif flag==2
        if PlotaMapas                                        % Se for para plotar os mapas. Caso negativo, não perde tempo !!
            % Resgata entradas do bloco para poder atualizar todos os mapas
            % Dados da Operação
            FreqOperacao=u(1);                        % Frequencia de operação [Hz] 
            PChegada=u(2)*1.019716;             % Pressão de Chegada convertida de bar para Kgf/cm2
            PSuc=u(3)*1.019716;                      % Pressão de Sucção convertida de bar para Kgf/cm2
            VazaoOleo=u(4);                              % Vazão em m3/d
            
            % Dados do otimizador
            FreqOtima=u(5);                                % Frequencia proposta pelo Controlador
            PChegadaOtima=u(6)*1.019716;    % Pressão de Chegada dada pelo otimizador  convertida de bar para Kgf/cm2
            PSucOtima=u(7)*1.019716;             % Pressão de Sucção dada pelo otimizador convertida de bar para Kgf/cm2
            VazaoOleoOtima=u(8);                     % Vazão dada pelo otimizador

            % Dados desejados pela engenharia (Alvos)
            FreqAlvoENG=u(9);                                  % Frequência Alvo ENG atual
            PMonAlvoENG=u(10)*1.019716;              % PMon Alvo ENG atual, convertida de bar para Kgf/cm2
            PSucAlvoENG=Interpola(FreqAlvoENG, PMonAlvoENG,MatrizSimulador,8);    % Estima em função da Freq e Pressao
            VazaoAlvoENG=Interpola(FreqAlvoENG, PMonAlvoENG,MatrizSimulador,3);  % Estima em função da Freq e Pressao

            % Monta vetores sempre na sequencia de [Medido,   AlvoENG,  Otima]
            PChegada= [PChegada,        PMonAlvoENG,    PChegadaOtima];
            PSuc=         [PSuc,                 PSucAlvoENG,      PSucOtima];
            Vazao=       [VazaoOleo,        VazaoAlvoENG,    VazaoOleoOtima];
            Freq=          [FreqOperacao,  FreqAlvoENG,       FreqOtima];

            % Atualiza pontos nos 3 mapas de operação
            AtualizaMapa(1,OperacaoMapa1,AlvoENGMapa1,SetPointMapa1,HabilitaRastro,Freq,PChegada);
            AtualizaMapa(2,OperacaoMapa2,AlvoENGMapa2,SetPointMapa2,HabilitaRastro,Freq,PSuc);
            AtualizaMapa(3,OperacaoMapa3,AlvoENGMapa3,SetPointMapa3,HabilitaRastro,Freq,Vazao);
            
            % Atualiza valores nos títulos dos gráficos
            TituloMapa1.String=strcat("PChegada Medida=",num2str(PChegada(1),'%.1f'),"    (*) SetPoint=",num2str(PChegada(3),'%.1f'),"     Erro = ",num2str(PChegada(1)-PChegada(3),'%.1f')," [Kgm/cm2]");
            SubTituloMapa1.String=strcat("                                          (x) Alvo ENG=",num2str(PChegada(2),'%.1f'),"    Erro =",num2str(PChegada(1)-PChegada(2),'%.1f')," [Kgm/cm2]");

            TituloMapa2.String=strcat("PSuc Medida=",num2str(PSuc(1),'%.1f'),"    (*) SetPoint= ",num2str(PSuc(3),'%.1f'),"     Erro =",num2str(PSuc(1)-PSuc(3),'%.1f')," [Kgm/cm2]");
            SubTituloMapa2.String=strcat("                                    (x) Alvo ENG=",num2str(PSuc(2),'%.1f'),"     Erro =",num2str(PSuc(1)-PSuc(2),'%.1f')," [Kgm/cm2]");

            TituloMapa3.String=strcat("Vazão Estimada= ",num2str(Vazao(1),'%.2f'),"    (*) SetPoint=",num2str(Vazao(3),'%.2f'),"     Erro =",num2str(Vazao(1)-Vazao(3),'%.2f')," [m3/dia]");
            SubTituloMapa3.String=strcat("                                    (x) Alvo ENG=",num2str(Vazao(2),'%.2f'),"     Erro = ",num2str(Vazao(1)-Vazao(2),'%.2f')," [m3/dia]");
            
            % Atualiza curva de isométrica da vazão correspondente ao alvo da ENG que pode ter alterado
            VarX=TabelaIsometricas.FreqBCSS;                           % Variável que vai compor o eixo X do Mapa   
            VarY=TabelaIsometricas.PressChegada;                    % Variável que vai compor o eixo Y do Mapa
            VarProcurada=TabelaIsometricas.VazaoOleo;           % Variável que vai compor a curva isométrica no Mapa
%            [VarX,VarY]=IsoCurva(VarX,VarY,VarProcurada,VazaoAlvoENG,VazaoAlvoENG/500,1);    % Tabela, Variável X, Variável Y, Variável procurada, Valor procurado, Tolerância, Escala
            [VarX,VarY]=IsoCurva(VarX,VarY,VarProcurada,VazaoAlvoENG,0.5,1);    % Tabela, Variável X, Variável Y, Variável procurada, Valor procurado, Tolerância, Escala
            set(IsoVazaoENG,'xdata',VarX,'ydata',VarY)                % Atualiza no mapa a curva isométrica da vazão 
            
           drawnow;
        end
        sys=[];
    end
end
%==================================================================================
%% ==================================================================================
function  AtualizaMapa(Mapa,OperacaoMapa,AlvoENGMapa,SetPointMapa,Rastro,Freq,Variavel)
  % Freq sempre na sequencia: [ Medido,   AlvoENG]
  % Variável PChegada, PSuc e Vazao, sempre na sequencia: [ Medido,   AlvoENG,  Otima]
    subplot(3,1,Mapa)   % Resgata mapa que será atualizado
    
    if Rastro
        % Resgata valores anteriores para poder plotar rastro
        FreqOperacaoAntes=OperacaoMapa.XData;    % Ponto da frequencia de Operação antes de atualizar
        FreqAlvoAntes=AlvoENGMapa.XData;                % Ponto da frequencia Alvo da ENG antes de atualizar
        Variavel1Antes=OperacaoMapa.YData;              % Ponto da Variável medida antes de atualizar
        Variavel2Antes=AlvoENGMapa.YData;                % Ponto da Variável ALVO da ENG antes de atualizar
        Variavel3Antes=SetPointMapa.YData;                 % Ponto da Variável Ótima (dada pelo otimizador) antes de atualizar
        
        % Plota os rastros
        plot([ FreqOperacaoAntes  Freq(1)],[Variavel1Antes  Variavel(1)],'k')       % Rastro do ponto de operação
%         plot([ FreqAlvoAntes  Freq(2)],[Variavel2Antes  Variavel(2)],'b')            % Rastro do alvo desejado pela ENG
%         plot([ FreqAlvoAntes  Freq(3)],[Variavel3Antes  Variavel(3)],'k')             % Rastro do set point dado pelo otimizador
    end

    % Atualiza pontos no respetivo mapa
    set(OperacaoMapa, 'xdata',Freq(1), 'ydata', Variavel(1));    % Atualiza ponto de operação
    set(SetPointMapa, 'xdata', Freq(3), 'ydata', Variavel(3));      % Atualiza Set Point dado pelo otimizador
%     if Mapa~=2       % Se não quiser plotar ENG no Mapa de PSuc
        set(AlvoENGMapa, 'xdata', Freq(2), 'ydata', Variavel(2));     % Atualiza alvo desejado pela engenharia
%     end
%     ax=gca;
%     ax.Subtitle.HorizontalAlignment='right';
end

%% ==================================================================================
function monta_mapas(MatrizSimulador,MatrizSimuladorContas,BTP,TabelaIsometricas,MargemPercentual)
    close all
    figure(1)
    set(gcf,'position',[20   5   570   875]);
    % set(gcf,'MenuBar','none');
    set(gcf,'name','MAPAS DE OPERAÇÃO');

    [Qdt, Qut,Condicao,Cor]=AvaliaCondicao(MatrizSimulador,BTP);
    gridFreq=unique(MatrizSimulador(:,1));   % Verifica o grid da frequencia
    Tam=0.1*10/height(gridFreq);       % Usa o tamanho do grid para propor o tamanho do cículo na plotagem

    % Varre grid definido para a frequência e calcula os limites de proteção
    for i=1:height(gridFreq)
         [QMin(i), QMax(i),PSucMin(i),PSucMax(i), PChegadaMin(i),PChegadaMax(i)]=ProtecoesMapas(MatrizSimuladorContas,BTP,gridFreq(i));
    end

    % =================================================
    % Monta as figuras para os mapas
    subplot(3,1,1) % Mapa de Pressão de Chegada x Frequência 
    hold on
    grid on
    % axis([ 39.9   60   10   50])
    % xlabel('Frequencia [Hz]');
    ylabel('Pressão de Chegada [Kgf/cm^2]');
    scatter(MatrizSimulador(:,1),MatrizSimulador(:,2),Tam*MatrizSimulador(:,3),Cor,'filled')
    colormap(prism)
    % Traça limites do mapa
    plot(gridFreq,PChegadaMin,'r')
    plot(gridFreq,PChegadaMin*(1+MargemPercentual/100),'r--')
    plot(gridFreq,PChegadaMax,'b')
    plot(gridFreq,PChegadaMax*(1-MargemPercentual/100),'b--')
    eixo=axis;
    eixo(1)=39.9;
    axis(eixo);         % Drible para efeito visual na escala do plot 40Hz
    
    % Isocurvas do mapa 1
    %  Completa mapa com curvas de isovazão
    for i=250:50:500
        VarX=TabelaIsometricas.FreqBCSS;                           % Variável que vai compor o eixo X do Mapa   
        VarY=TabelaIsometricas.PressChegada;                    % Variável que vai compor o eixo Y do Mapa
        VarProcurada=TabelaIsometricas.VazaoOleo;           % Variável que vai compor a curva isométrica no Mapa
        [VarX,VarY]=IsoCurva(VarX,VarY,VarProcurada,i,0.5,1);    % Tabela, Variável X, Variável Y, Variável procurada, Valor procurado, Tolerância, Escala
        plot(VarX,VarY,'k:')
        text(max(VarX),max(VarY)-1,strcat(num2str(i),"m^3/d"),'FontSize',8)
    end

    % =============
    subplot(3,1,2)    % Mapa de Pressão de Sucção x Frequência
    hold on
    grid on
    % axis([ 39.9  60   65   105])
    % xlabel('Frequencia [Hz]');
    ylabel('Pressão de Sucção [Kgf/cm^2]');
    scatter(MatrizSimulador(:,1),MatrizSimulador(:,8),Tam*MatrizSimulador(:,3),Cor,'filled')
    colormap(prism)
    % Traça limites do mapa
    plot(gridFreq,PSucMin,'r')
    plot(gridFreq,PSucMin*(1+MargemPercentual/100),'r--')
    plot(gridFreq,PSucMax,'b')
    plot(gridFreq,PSucMax*(1-MargemPercentual/100),'b--')
    eixo=axis;
    eixo(1)=39.9;
    axis(eixo);         % Drible para efeito visual na escala do plot 40Hz

   % Completa mapa 2 com curvas isobáricas da PChegada
    for i=20:10:50
        VarX=TabelaIsometricas.FreqBCSS;                          % Variável que vai compor o eixo X do Mapa   
        VarY=TabelaIsometricas.PressSuccao;                      % Variável que vai compor o eixo Y do Mapa
        VarProcurada=TabelaIsometricas.PressChegada;    % Variável que vai compor a curva isométrica no Mapa
        [VarX,VarY]=IsoCurva(VarX,VarY,VarProcurada,i,0.5,1);    % Tabela, Variável X, Variável Y, Variável procurada, Valor procurado, Tolerância, Escala
        plot(VarX,VarY,'k:')
        text(min(VarX),max(VarY)-1,strcat(num2str(i)),'FontSize',8)
    end

    % =============
    subplot(3,1,3)    % Mapa de Vazão x Frequência
    hold on
    grid on
    % axis([ 39.9  60   65   105])
    xlabel('Frequencia [Hz]');
    ylabel('Vazão de Óleo [m3/dia]');
    scatter(MatrizSimulador(:,1),MatrizSimulador(:,3),Tam*MatrizSimulador(:,3),Cor,'filled')
    colormap(prism)
    % Traça limites do mapa
    plot(gridFreq,QMin,'b')
    plot(gridFreq,QMin*(1+MargemPercentual/100),'b--')
    plot(gridFreq,QMax,'r')
    plot(gridFreq,QMax*(1-MargemPercentual/100),'r--')
    eixo=axis;
    eixo(1)=39.9;
    axis(eixo);         % Drible para efeito visual na escala do plot 40Hz
    
   % Completa mapa 3 com curvas isobáricas da PChegada
    for i=20:10:50
        VarX=TabelaIsometricas.FreqBCSS;                          % Variável que vai compor o eixo X do Mapa   
        VarY=TabelaIsometricas.VazaoOleo;                         % Variável que vai compor o eixo Y do Mapa
        VarProcurada=TabelaIsometricas.PressChegada;    % Variável que vai compor a curva isométrica no Mapa
        [VarX,VarY]=IsoCurva(VarX,VarY,VarProcurada,i,0.5,1);    % Tabela, Variável X, Variável Y, Variável procurada, Valor procurado, Tolerância, Escala
        plot(VarX,VarY,'k:')
        text(min(VarX),min(VarY)-1,strcat(num2str(i)),'FontSize',8)
    end

end
%==================================================================================
