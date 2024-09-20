% Para propor uma trajetória em uma rampa de aceleração

clc
clear all
close all

%% =============================================================================
% Carrega as tabelas de referências da Petrobras para as proteções e para
% buscar condiçoes iniciais em pontos de operação reais
CarregaTabelas; 

Plano=readtable('PlanoAceleracao.xlsx');     % Tabela com plano para ser atualizado 
% Carrega 1 registro da tabela atual apenas para ter a estrutura e inserir registros
NovoPlano=Plano(1,:);


FreqAlvo   = [ 41  42  43   44   45   46   47   48   49   50   51   52   53   54   55   55   55]; 
PMonAlvo = [ 20  20  22   22   22   22   25   25   25   25   25   25   25   25   27   27   32];
Tam=width(FreqAlvo);
Ponto=0:Tam-1;
Tempo=450*Ponto;

% for i=1:Tam
%     NovoPlano(i,:)=[Ponto(i),  FreqAlvo(i), PMonAlvo(i), Tempo(i)];
% end

% i=1;
% for FreqAlvo=40:55
%      Limites=CalcLimites(FreqAlvo,TabSimulador,BTP,TabRestricoesDinamicas,FxPercent,ProtecaoFixa);    % Extrai limites
%      PMonAlvo(i)=Limites.ProductionSurfacePressure(2);      % Extrai ponto limite minimo da PChegada (em bar)
%      PMonAlvo(i)=max(PChegadaMinima,PMonAlvo(i));          % Considera o limite da PChegada no mapa ou um valor pré-estabelecido
%      i=i+1;
% end
