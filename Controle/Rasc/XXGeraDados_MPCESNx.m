% =============================================================================
% Gera dados MPCESNx em combinação com os parâmetros de sintonia
% =============================================================================
Hp_MPC =  [6, 8, 10];               %Dim = 3
Hc_MPC =  [2, 4, 6, 8, 10];        %Dim = 5
r_FREQ =  [1, 2, 3, 4, 5];         %Dim = 5
r_PMON =  [1, 0.7, 0.5, 0.3, 0.1]; %Dim = 5    registros (3x4x5x5 = 375) 

% Numero de combinações 
NumC=width(Hp_MPC)*width(Hc_MPC)*width(r_FREQ)*width(r_PMON);

c=0; % Inicializa contador
 
for i=1:width(Hp_MPC)
    for j=1:width(Hc_MPC)
        for k=1:width(r_FREQ)
            for z = 1:width(r_PMON)
                c=c+1;                       % Atualiza contador
                disp(strcat(num2str(c),"/",num2str(NumC)));
                Hp =  Hp_MPC(i);                 % Horizonte de predição
                Hc =  Hc_MPC(j);                 % Horizonte de controle
                r=    [r_FREQ(k) r_PMON(z)];              % Matriz de supressão na entrada
                xmk_MPC = repmat([YIni(1);YIni(2);YIni(3);YIni(4);YIni(5);YIni(6);YIni(7);YIni(8);YIni(9);YIni(10)],1,Hp+1); %condição inicial dos estados 
                du0  = zeros(Hc*size(dumax,1),1);
                X0Du0YsP = [xmk_MPC(:);du0;[xmk_MPC(1);xmk_MPC(2)]];      % estimativa inicial para o solver (X_k;Deltau_k;Ysp_k) calcular as variáveis de decisão
                save('xmk_MPC','X0Du0YsP');
                NomeArq=monta_nome(Hp_MPC(i),Hc_MPC(j),r_FREQ(k),r_PMON(z));
                disp(NomeArq)  
                out=sim('MPC_ESNx_JUB27.slx',Time_Stop-1);                 % Usa ambiente do Simulink para gerar dados simulados MPC
                MPC_ESNx=array2table(out.MPC_ESNx);
                for p=1:width(Nomes)                                       % Varre as colunas para atualizar nomes
                    MPC_ESNx.Properties.VariableNames(p)=Nomes(p);         % Atualiza nome de colunas na Tabela
                end
                NomeA=strcat(LocalResultado,"\MPC_ESNx_",NomeArq,".parquet");
                parquetwrite(NomeA,MPC_ESNx);
    
            end
        end
    end
end

disp('FIM DA SIMULAÇÃO !!!')
