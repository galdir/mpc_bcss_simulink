%Loop de simulações para diferentes HP
clc
clear all
close all
for Hp=3:10
    CarregaDadosMPC;
    out=sim('MPC_JUB27.slx');
    SalvaSimulacao
    clc
    close all
end