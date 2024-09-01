function NomeArq=monta_nome(tam_Hp,tam_Hc,tam_rFREQ,tam_rPMON);
NomeArq="MPC";
NomeArq=strcat(NomeArq,"_Hp",num2str(tam_Hp));
NomeArq=strcat(NomeArq,"_Hc",num2str(tam_Hc));
NomeArq=strcat(NomeArq,"_rFREQ",num2str(tam_rFREQ));
NomeArq=strcat(NomeArq,"_rPMON",num2str(tam_rPMON));