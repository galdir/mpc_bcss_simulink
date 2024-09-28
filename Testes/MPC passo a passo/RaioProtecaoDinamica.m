function R=RaioProtecaoDinamica(x,y);
    % Função para definir um tamanho de raio de proteção do robô em função da sua posição (coordenadas x,y)
%     R=0.1;

    RIni=1.3;
    RFim=0.1;
    Xmin=-2;
    Xmax=2;
    a=(RFim-RIni)/(Xmax-Xmin);
    b=RFim-a*Xmax;
    R= a*x+b;    % Raio de RIni a RFim em função da coordenada Xmin até Xmax

end
