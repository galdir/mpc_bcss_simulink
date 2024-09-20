% Para Simulação automática
[YIni,UIni]=SelCondicaoInicial(1);


% =============================================================================
% Define Nomes para salvar na tabela (.parquet)
% Nomes_Processo =   {'PSuc','PChegada','PDiff','PDescarga','TMotor','ITorque','ITotal','TSuc','Vibracao','TChegada','Tempo_modelo'};
% Nomes_MPC=         {'FREQ_ALVO','PMON_ALVO','FREQ','PMON','Freq_max','PMON_max','Freq_min','PMON_min',...
%                     'ysp*_Psuc','ysp*_Pche','Psuc_max_mapa','Pcheg_max_mapa','Psuc_min_mapa','Pcheg_min_mapa',...
%                     'Psuc_H','Pcheg_H','Pdif_H','Pdesc_H','Tmotor_H','Ctorque_H','CTotal_H','Tsuc_H','Vib_H','Tche_H',...
%                     'Psuc_L','Pcheg_L','Pdif_L','Pdesc_L','Tmotor_L','Ctorque_L','CTotal_L','Tsuc_L','Vib_L','Tche_L',...
%                     'Psuc_pred','Pcheg_pred','Pdif_pred','Pdesc_pred','Tmotor_pred','Ctorque_pred','CTotal_pred','Tsuc_pred','Vib_pred','Tche_pred',...
%                     'Tempo_solver','dumax_FREQ','dumax_PMON','Hp','Hc','q_psuc','q_pcheg','r_FREQ','r_PMON','qu_FREQ','qu_PMON'};
% Nomes = [Nomes_Processo Nomes_MPC];
% LocalResultado=strcat(".\Resultados"); %salva os arquivos na pasta "Resultados"


% Inicializar LSTM ...



%GeraDados_MPCLSTM
%GeraDados_MPCESNx

% % =============================================================================
% % Roda simulação ESNx
% out=sim('MPC_ESNx_JUB27.slx',Time_Stop);                   % Usa ambiente do Simulink para gerar dados simulados e avaliar o erro cometido pela rede ESN
% MPC_ESNx=array2table(out.MPC_ESNx);
% for i=1:width(Nomes)                                       % Varre as colunas para atualizar nomes
%     MPC_ESNx.Properties.VariableNames(i)=Nomes(i);         % Atualiza nome de colunas na Tabela ESNx
% end
% NomeA=strcat(LocalResultado,"\MPC_ESNx",".parquet");
% parquetwrite(NomeA,MPC_ESNx);
% disp(strcat("Arquivos salvos em formato parquet na pasta ",LocalResultado))
% 
% % =============================================================================
% % Roda simulação LSTM
% out=sim('MPC_LSTM_JUB27.slx',Time_Stop);                   % Usa ambiente do Simulink para gerar dados simulados
% MPC_LSTM=array2table(out.MPC_LSTM);
% for i=1:width(Nomes)                                       % Varre as colunas para atualizar nomes
%     MPC_LSTM.Properties.VariableNames(i)=Nomes(i);         % Atualiza nome de colunas na Tabela ESNx
% end
% NomeA=strcat(LocalResultado,"\MPC_LSTM",".parquet");
% parquetwrite(NomeA,MPC_LSTM);

disp('FIM DA SIMULAÇÃO !!!')
