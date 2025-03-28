function [Qdt, Qut,Condicao,Cor]=AvaliaCondicao(T,BTP);
% Recebe a tabela T e as condições do BTP. Com base nisso, 
% reproduz as contas para análise das condições de Up e Downthrust
% conforme contas do Anexo C1

% Recebida a variável T que corresponde a matriz do simulador Petrobras
% Coluna1 = FreqBCSS
% Coluna2=PressChegada
% Coluna3=VazaoOleo
% Coluna4=VazaoLiquido
% Coluna5=Twh
% Coluna6=Pwh
% Coluna7=DeltaP
% Coluna8=PressSuccao

% Extrai valores de API, dg, pb, BSW e RGO do último teste de produção
API=BTP(1);    dg=BTP(2);    BtpPb=BTP(3);   BSW=BTP(4);   BtpRGO=BTP(5);

% (1/0) para definir se quer mostrar os resultados das contas na tela. Útil na fase de depuração e conferência das contas
%OBS: Para visualização das contas, habilite a visualização de apenas um por vez, ou seja, a MEMÓRIA DE CÁLCULO ou os RESULTADOS do Anexo C1

mostra_memoria=0;         % Ilustra os resultados das contas referentes a MEMÓRIA DE CÁLCULO do Anexo C1
mostra_resultados=0;      % Ilustra os resultados das contas referentes aos RESULTADOS do Anexo C1

for i=1:height(T)      %  Para cada registro da tabela T recebida (pode ser 1 registro ou a tabela completa)
    % Contas que reproduzem a MEMÓRIA DE CÁLCULO do Anexo C1
    QL=T(i,3)/(100-BSW)*100;
    do=141.5/(131.5+API);
    Ppc=708.75-57.5*dg;
    Tpc=169+314*dg;
    Psuc=(T(i,8)/1.01972)*14.50377+14.7;
    Tsuc=(T(i,5)+273.15)*9/5;
    RGO=BtpRGO/0.1781;
    Pb=BtpPb*14.22334+14.7;
    if Psuc<=Pb
        Rs1=dg*((Psuc/18.2+1.4)*10^(0.0125*API-0.00091*(Tsuc-459.67)))^1.2048;
    else
        Rs1=RGO;
    end
    Rs2=Rs1*0.1781;
    Bo=0.972+0.000147*(Rs1*(dg/do)^0.5+1.25*(Tsuc-460))^1.175;
    Ppr=Psuc/Ppc;
    Tpr=Tsuc/Tpc;
    z=1-3.52*Ppr/(10^(0.9813*Tpr))+0.274*Ppr^2/(10^(0.8157*Tpr));
    Bg=(14.7/Psuc)*(Tsuc*z)/520;
    Bw=(1+(-1.0001*10^-2+1.33391*(10^-4)*(Tsuc-459.67)+5.50654*(10^-7)*(Tsuc-459.67)))*(1+(-1.95301*(10^-9)*Psuc*(Tsuc-459.67)-1.72834*(10^-13)*(Psuc^2)*(Tsuc-459.67)-3.58922*(10^-7)*Psuc-2.25341*(10^-10)*(Psuc^2)));

    % Contas que reproduzem os RESULTADOS do Anexo C1
    QoPT=T(i,3)*Bo;
    QwPT=QL*(BSW/100)*Bw;
%    QgPT=(BtpRGO-Ppr)*T(i,3)*Bg;  % Linha antiga
    QgPT=(BtpRGO-Rs2)*T(i,3)*Bg;
    Qtotal=QoPT+QwPT+QgPT;
    FGL=QgPT/Qtotal*100;
    Qdt(i)=3021*T(i,1)/60;
    Qut(i)=5564*T(i,1)/60;
    
    % Verifica condição de vazão para Up e Downthrust
    % Por padrao, assume condição Normal
    Condicao(i)="Normal";  Cor(i)=2;
    if Qtotal>=Qut(i)
        Condicao(i)="Upthrust"; Cor(i)=0;
    end
    if Qtotal<=Qdt(i)
        Condicao(i)="Downthrust"; Cor(i)=1;
    end
      
        % ======================================================================
    % ======================================================================
    % As proximas linhas de código são apenas para a possibilidade de
    % ilustrar os resultados na tela e conferir as contas do Anexo C1
    % ÚTIL NA FASE DE DEPURAÇÃO
    
    if mostra_memoria            % Se for para mostrar as contas da MEMÓRIA DE CÁLCULO do Anexo C1
        % Prepara cabeçalho da tabela que será ilustrada na tela
        if i==1    % Se for o primeiro registro, limpa a tela e monta o cabeçalho
            fprintf('%s\n'," Hz    P.Cheg   Psuc     QL        do      Ppc      Tpc       Psuc      Tsuc     RGO    Pb          Rs       Rs      Bo      Ppr    Tpr     z         Bg           Bw")
            fprintf('%s\n',"--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")   
        end
        % Mostra valores calculados
        fprintf('%2.1f     %3.1f    %3.2f   %4.1f   %4.3f   %4.1f   %4.1f   %5.2f   %4.1f   %4.1f   %5.1f   %4.1f  %4.2f  %3.3f   %3.2f  %3.2f  %2.3f  %2.5f  %2.6f\n',T(i,1),T(i,2), T(i,3),QL,do,Ppc,Tpc,Psuc,Tsuc,RGO,Pb,Rs1,Rs2,Bo,Ppr,Tpr,z,Bg,Bw);
    %                                                       QL         do        Ppc       Tpc     Psuc    Tsuc     RGO      Pb        Rs       Rs        Bo       Ppr       Tpr         z        Bg        Bw  
    end
    
    if mostra_resultados            % Se for para mostrar as contas dos RESULTADOS do Anexo C1
        % Prepara cabeçalho da tabela que será ilustrada na tela
        if i==1    % Se for o primeiro registro, limpa a tela e monta o cabeçalho
            fprintf('%s\n'," Hz    P.Cheg   Psuc    QoPT    QwPT   QgPT    FGL  Qdown    Qtotal       Qup        Condicao")
            fprintf('%s\n',"--------------------------------------------------------------------------------------------------------------------------------")   
        end
        % Mostra valores calculados
         fprintf('%2.1f     %3.1f    %3.2f    %4.1f   %4.1f   %4.1f   %4.1f    %5.1f    %5.1f     %5.1f     %s\n',T(i,1),T(i,2),T(i,3),QoPT,QwPT,QgPT,FGL,Qdt(i),Qtotal,Qut(i),Condicao(i));
         %            Hz    P.Cheg     Psuc   QoPT  QwPT   QgPT    FGL    Qdown   Qtotal      Qup     Condicao
    end
end

