%  Rotina para salvar resultados de simulação em arquivo
Qy=Qy*diag([LimitesMax(2)   LimitesMax(11)]);   % Desfaz contas de ponderação para saber Qy original
Qy=Qu*diag([ umax(1)           umax(2)  ]);              % Desfaz contas de ponderação para saber Qu original

DadosProcesso=TabelaDadosProcesso(out.DadosProcesso);
SaidaMPC=TabelaSaidasMPC(out.SaidaMPC,nx,Hp);

NomeArq=strcat("Simula Hp ",num2str(Hp));
save(NomeArq,'out','ny','nx','nu','Qy','Qu','Qx','R', 'Hp', 'Hc','umin','umax', 'dumax','SNR','LimitesMin','LimitesMax',...
    'DataHoraIni','WallTime','NomeESN','DadosProcesso','SaidaMPC');

TabelaCompleta=horzcat(SaidaMPC,DadosProcesso);
TabelaCompleta.('Tempo')=out.tout;
 writetable(TabelaCompleta,strcat(NomeArq,'.xlsx')); 

%% ===========================   FIM DO PROGRAMA PRINCIPAL ==============================
%% ======================================================================================
% Função para criar tabela com dados do processo
function DadosProcesso=TabelaDadosProcesso(DadosProcesso)
    DadosProcesso=array2table(DadosProcesso);
    Nomes={'PSuc','PChegada','PDiff','PDescarga','TMotor','ITorque','ITotal','TSuc','Vibracao','TChegada','Vazao',...
        'FreqSetAplicada','PMonSetAplicada','TempoESN'};
    DadosProcesso.Properties.VariableNames=Nomes;    
end
%% ======================================================================================
% Função para criar tabela com dados da saida do MPC
function SaidaMPC=TabelaSaidasMPC(SaidaMPC,nx,Hp)
    SaidaMPC=array2table(SaidaMPC);
    Variaveis={'PSuc','PChegada','PDiff','PDescarga','TMotor','ITorque','ITotal','TSuc','Vibracao','TChegada','Vazao'};
    Nomes={'Feasible',"NumIteracoes","TempoSolver","FreqSetProposta","PMonSetProposta", "DeltaFreqProposto","DeltaPMonProposto",...
        'SomaDeltaFreq',"Jy","Ju","Jx","Jr",...
        'Ex_PSuc','Ex_PChegada','Ex_PDiff','Ex_PDescarga','Ex_TMotor','Ex_ITorque','Ex_ITotal','Ex_TSuc','Ex_Vibracao',...
        'Ex_TChegada','Ex_Vazao','Ey_PChegada','Ey_Vazao',"SetPChegada","SetVazao"};
    
    % Compõe nome para os estados atuais e no horizonte futuro de 1 até Hp
    for k=1:1+Hp
        for i=1:nx
            NomeVar=strcat(num2str(k-1),"_",Variaveis(i));
            Nomes=[Nomes, NomeVar];
        end
    end
        % Compõe nome para as ações de controle de 1 até Hp
    for k=1:Hp
            Nome1=strcat(num2str(k),"_FreqAlvo");
            Nome2=strcat(num2str(k)," _PMonAlvo");
            Nomes=[Nomes, Nome1,   Nome2];
    end
    
    SaidaMPC.Properties.VariableNames=Nomes;
end
%% ======================================================================================