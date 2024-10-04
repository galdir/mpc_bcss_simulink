import casadi.*;

CarregaTabelas; 
disp(TabelaLimitesDinamicos.Properties.VariableNames);

matrizLimitesDinamicos = table2array(TabelaLimitesDinamicos(:, [1,3:end])); %cortando a coluna LIMITES

freq_sym = MX.sym('Freq_sym',1);
buscaRestricoesLimites_casadi_sym = buscaLimitesTabela_casadi(matrizLimitesDinamicos, freq_sym);
f_buscaRestricoesLimites_casadi_sym = Function('f_lim', {freq_sym}, {buscaRestricoesLimites_casadi_sym});

%buscando limites dinamicos
freq = 55;
limites = f_buscaRestricoesLimites_casadi_sym(freq);
disp(limites);