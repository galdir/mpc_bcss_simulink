%  Rotina para salvar resultados de simulação em arquivo
QY=Qy*diag([LimitesMax(2)   LimitesMax(11)]);   % Desfaz contas de ponderação para saber Qy original
QU=Qu*diag([ umax(1)           umax(2)  ]);              % Desfaz contas de ponderação para saber Qu original
Resultado=CriaTabela(out.Resultado,nx,nu,Hp);   % Já coloco em formato de tabela para não ter problema de identificar colunas
NomeArq=MontaNome(QY, QU,Hp,Hc);
save(NomeArq,'Qy','Qx','Qu','R', 'Hp', 'Hc','umin','umax', 'dumax','SNR','MargemPercentual','Resultado');

%% ===========================   FIM DO PROGRAMA PRINCIPAL ==============================
%% ======================================================================================
% Função para criar tabela com dados de simulação, com objetivo de ter os nomes das colunas
function Resultado=CriaTabela(Resultado,nx,nu,Hp)
    Resultado=array2table(Resultado);
    Nomes={'PSuc','PChegada','PDiff','PDescarga','TMotor','ITorque','ITotal','TSuc','Vibracao','TChegada','Vazao',...
        'FreqSetAplicada','PMonSetAplicada',...
        'Feasible',"N.Iteracoes","T.Solver","FreqSetProposta","PMonSetProposta", "DeltaFreqProposto","DeltaPMonProposto",...
        'SomaDeltaFreq',"Jy","Ju","Jx","Jr",...
        'Ex_PSuc','Ex_PChegada','Ex_PDiff','Ex_PDescarga','Ex_TMotor','Ex_ITorque','Ex_ITotal','Ex_TSuc','Ex_Vibracao',...
        'Ex_TChegada','Ex_Vazao','Ey_PChegada','Ey_Vazao',"SetPChegada","SetVazao"};
    
    % Compõe nome para os estados atuais e no horozonte futuro de 1 até Hp
    for k=1:1+Hp
        for i=1:nx
            NomeVar=strcat(num2str(k-1)," - ",Nomes(i));
            Nomes=[Nomes, NomeVar];
        end
    end
    
    % Compõe nome para as ações de controle de 1 até Hp
    for k=1:Hp
            Nome1=strcat(num2str(k)," - FreqAlvo");
            Nome2=strcat(num2str(k)," - PMonAlvo");
            Nomes=[Nomes, Nome1,   Nome2];
    end
    
    Resultado.Properties.VariableNames=Nomes;


end
%% ======================================================================================
% Função para compor nome do arquivo de simulação
function NomeArq=MontaNome(Qy, Qu,Hp,Hc)
    NomeArq=strcat("Simula  Hp-Hc=",num2str(Hp),"-",num2str(Hc),"   Qy=[",num2str(Qy(1,1)),"  ",num2str(Qy(2,2))," ]    Qu=[",num2str(Qu(1,1)),"  ",num2str(Qu(2,2))," ]");
end
