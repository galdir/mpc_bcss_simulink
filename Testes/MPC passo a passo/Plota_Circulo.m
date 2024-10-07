function h = Plota_Circulo(x,y,r,solido,cor)
if nargin<1
    x=1.5;
    y=1;
    r=0.2;
    solido=1;
    cor='rx';
end

% Calcula pontos que compõe o círculo
[px  py]=Pontos_circulo(x,y,r);
% plota circulo
hold on
if solido
    cor=cor(1);    % Só a cor, sem eventual característica do tipo de plot
    h=fill(px, py,cor);
else
    h = plot(px, py,cor);
end
end

