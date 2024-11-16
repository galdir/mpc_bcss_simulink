clc
clear
close all


%% =============================================================================
% Carrega as tabelas de referências da Petrobras para as proteções e para
% buscar condiçoes iniciais em pontos de operação reais
CarregaTabelas;


MatrizSimulador = table2array(TabSimulador);
MatrizVazao = table2array(TabSimulador(:,1:3));

import casadi.*
Freq_sym = MX.sym('F',1);
Press_sym = MX.sym('P',1);
Interpola_casadi_vazao_sym=Interpola_casadi_vazao(Freq_sym, Press_sym, MatrizVazao);
f_Interpola_casadi_vazao_sym = Function('f', {Freq_sym, Press_sym}, {Interpola_casadi_vazao_sym});

freq = 48.3;
pmon = 35;

%tic

vasao_casadi = full(f_Interpola_casadi_vazao_sym(freq, pmon));
disp('vasao interpola casadi')
disp(vasao_casadi);
%toc

%testa Interpola original
%tic
vasao = Interpola(freq, pmon, MatrizSimulador, 3);
disp('vasao interpola original')
disp(vasao)
%toc

% testa valroes de freq para comparar interpola_casadi e original
%variando frequencia
dif_vazoes=[];

cont = 1;
for  freq = 0:0.1:100
    %disp(cont)
    vazao_casadi = full(f_Interpola_casadi_vazao_sym(freq, pmon));
    if(vazao_casadi==0 || isnan(vazao_casadi))
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!vazao casadi com problema');
        beep;
        disp(vazao_casadi)
    end
    vazao = Interpola(freq, pmon, MatrizSimulador, 3);
    dif = vazao-vazao_casadi;
    %disp(dif)
    dif_vazoes(cont) = dif;
    cont = cont +1;

end
disp('soma diferenca de vazoes')
disp(sum(abs(dif_vazoes)))

%variando pressao
freq=44;
dif_vazoes=[];

cont = 1;
for  pmon = 0:0.1:100
    %disp(cont)
    vazao_casadi = full(f_Interpola_casadi_vazao_sym(freq, pmon));
    if(vazao_casadi==0 || isnan(vazao_casadi))
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!vazao casadi com problema');
        beep;
        disp(vazao_casadi)
    end
    vazao = Interpola(freq, pmon, MatrizSimulador, 3);
    dif = vazao - vazao_casadi;
    %disp(dif)
    dif_vazoes(cont) = dif;
    cont = cont +1;

end
disp('soma diferenca de vazoes')
disp(sum(abs(dif_vazoes)))



