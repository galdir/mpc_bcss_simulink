function Saidas=VerHistorico(Dados,nVar,Horizonte);
% Apenas reorganiza os dados para permitir a visualização, no Simulink, de cada uma das
% nVar variáveis, com seus respectivos Horizontes
Dados=reshape(Dados,nVar,Horizonte)';
Saidas=Dados(:);
