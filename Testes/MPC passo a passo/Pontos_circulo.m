function [px  py] = Pontos_circulo(x,y,r)
% Calcula pontos que compõe o círculo
th = 0:pi/50:2*pi;
px = r * cos(th) + x;
py = r * sin(th) + y;   