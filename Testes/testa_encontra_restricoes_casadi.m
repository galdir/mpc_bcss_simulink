CarregaTabelas;
matrizLimitesDinamicos = table2array(TabelaLimitesDinamicos(:, [1,3:end])); %cortando a coluna LIMITES
iFreq=1;
iTMot=2;
iTSuc=3;
iVib=4;
iCTot=5;
iCTor=6;
iPSuc=7;
iPDes=8;
iPDif=9;
iPChe=10;
iTChe=11;
iVaz=12;
matrizLimitesDinamicos = matrizLimitesDinamicos(:, [iFreq, iPSuc, iPChe, iPDif, iPDes, iTMot, iCTor, iCTot, iTSuc, iVib, iTChe, iVaz]);

import casadi.*

Freq_sym = MX.sym('Freq');
matrizLimitesDinamicos_sym = MX.sym('matrizLimitesDinamicos', size(matrizLimitesDinamicos));

% Chama Interpola_casadi com variáveis simbólicas
%New_sym = encontrarRestricoesTabela(matrizLimitesDinamicos, Freq_sym);
buscaLimitesMatriz_casadi_sym = buscaLimitesMatriz_casadi(matrizLimitesDinamicos, Freq_sym);

f = Function('f', {matrizLimitesDinamicos_sym, Freq_sym}, {buscaLimitesMatriz_casadi_sym});
Freq_real = 45;
limites = f(matrizLimitesDinamicos, Freq_real);
[lim] = full(limites);
lim(1,:)
lim(2,:)

