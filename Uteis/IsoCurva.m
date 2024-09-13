function [Iso_X,Iso_Y]=IsoCurva(VarX,VarY,PontosMapa,ValorProcurado,Tolerancia,Escala)
  % T é a tabela proveniente do simulador (ou refeita para aumentar a resolução)
  % VarX é a coluna associada a variável X que compõe o mapa onde será traçada a isocurva
  % VarY é a coluna associada a variável Y que compõe o mapa onde será traçada a isocurva
  % PontosMapa é a coluna que tem os valores para as isocurvas
  % ValorProcurado é o valor que será procurado na coluna PontosMapa
  % Tolerancia é o valor de tolerância para assumir que o valor procurado foi encontrado
  
    % Encontra indices na tabela T que tem o ValorProcurado, considerada a tolerância
   Iso_indexes=ismembertol(PontosMapa,ValorProcurado,Tolerancia,'DataScale',Escala);
   
    % Encontra pontos das variáveis X e Y correspondentes aos ínices encontrados
    Iso_X=VarX(Iso_indexes);   % Indices que compõe a variável do eixo X
    Iso_Y=VarY(Iso_indexes);   % Indices que compõe a variável do eixo X
end

