clc
close all
clear all

for Hp=3:10
    NomeArq=strcat("Simula Hp ",num2str(Hp));
    load(strcat(NomeArq,'.mat'))

    TabelaCompleta=readtable(strcat(NomeArq,'.xlsx'));

    Tabela=TabelaCompleta(TabelaCompleta.Feasible==1,:);
    NomeSel={'Tempo','FreqSetAplicada','PChegada','Jy','Ju','Jx','Jr'};
    Tabela=Tabela(:,NomeSel); 
    CustoTotal=Tabela.Jy+Tabela.Ju+Tabela.Jx+Tabela.Jr;
    Texto=strcat("Hp=",num2str(Hp));
    plot3(Tabela.FreqSetAplicada,Tabela.PChegada,CustoTotal,'DisplayName',Texto)
    if Hp==3    % Dados do gráfuico, só na 1a vez
        title("Custo com Qy= [1  10]   Qu=[10  1]")
        grid on
        hold on
        xlabel('Frequência [Hz]')
        ylabel('PChegada [bar]')
        zlabel('Custo Total')
        legend
  end
    % plot3(Tabela.FreqSetAplicada,Tabela.PChegada,Tabela.Jy,'DisplayName','Custo Qy')
    % plot3(Tabela.FreqSetAplicada,Tabela.PChegada,Tabela.Ju,'DisplayName','Custo Qu')
end