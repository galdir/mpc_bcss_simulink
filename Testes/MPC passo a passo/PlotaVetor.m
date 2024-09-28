% function PlotaVetor(A,x,y,Teta);
% Plota vetor de amplitude A com origem na coordenada (x,y) e ângulo Teta
close all

A=1;
x=2;
y=3;
Teta=pi/5;
xf=x+A*cos(Teta);
yf=y+A*sin(Teta);

plot([x xf ],[y yf],'b')
hold on
grid on

% % Fazer a seta na ponta do vetor
% As=A/20;     % Proporcionalmente, 1/10 do tamanho do vetor
% x1s=xf+As*cos(pi+Teta-0.3);
% y1s=yf+As*sin(pi+Teta-0.3);
% plot([xf x1s ],[yf y1s],'b')
% 
% x2s=xf+As*cos(pi+Teta+0.3);
% y2s=yf+As*sin(pi+Teta+0.3);
% plot([xf x2s ],[yf y2s],'b')

