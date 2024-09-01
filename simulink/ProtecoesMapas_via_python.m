function [QMin, QMax,PSucMin,PSucMax, PChegadaMin,PChegadaMax]=ProtecoesMapas_via_python(Tt,BTP,Freq,gridP)
% Para calcular os limites dos mapas para uma determinada condição de frequência
%
% Em fase de depuração, permite plotar a linha vertical do mapa, para a
% frequencia especifica definida
%
% IMPORTANTE:
% Para não confindir os limites que são plotados no mapa, é conveniente que  o parâmetro
% gridP seja menor ou igual ao grid de pressão definidopara gerar o mapa utilizado.
%

interpola = py.importlib.import_module('codigos_python.interpola');
py.importlib.import_module('builtins');
pandas = py.importlib.import_module('pandas');
tabelaSimulador_df = pandas.DataFrame(table_para_pydict(Tt));
BTP_df = pandas.DataFrame({table_para_pydict(BTP)});
avalia_condicao = py.importlib.import_module('codigos_python.avalia_condicao');

%======================================================
% Limites para as interpolações
PMin=min(Tt.PressChegada);       % Assume o valor minimo de PChegada existente na tabela do simulador
PMax=max(Tt.PressChegada);    % Assume o valor máximo de PChegada existente na tabela do simulador

% ======================================================
% Cria uma tabela temporária para avaliar as várias condições de Pressão de
% Chegada em função da Frequencia definida
T=Tt(1,:);    % Só para criar estrutura da tabela
i=1;   %   Contador
for Pchegada=PMin:gridP:PMax
    valores_interpolados_df = interpola.interpola(Freq,Pchegada,tabelaSimulador_df);
    valores_interpolados_table = pandas_para_MatlabTable(valores_interpolados_df);
    % Converte a matriz numpy para uma matriz MATLAB
    %valores_interpolados = double(valores_interpolados);
    T(i,:) = valores_interpolados_table;
    %T(i,:)=Interpola(Freq,Pchegada,Tt);    % Cria novo registro com base no valor interoplado
    i=i+1;    % Incrementa contador de registros
end

% Avalia condição de Up e Downthrust
T_df = pandas.DataFrame(table_para_pydict(T));
resultado_df = avalia_condicao.avalia_condicao(T_df, BTP_df);
resultado= pandas_para_MatlabTable(resultado_df);
Qdt = resultado.Qdt;
Qut = resultado.Qut;
Condicao = resultado.Condicao;
Cor = resultado.Cor;

%[Qdt, Qut,Condicao,Cor]=AvaliaCondicao(T,BTP);


% Define valores iniciais para os limites de Vazão, PSuc e PChegada
QMin=9999999;
QMax=0;
PSucMin=9999999;
PSucMax=0;
PChegadaMin=9999999;
PChegadaMax=0;

% Varre a tabela temporária para extrair os limites desejados
for i=1:height(T)
    if Condicao(i)=="Normal"
        %          QMin=min(QMin,T.VazaoLiquido(i,1));
        %          QMax=max(QMax,T.VazaoLiquido(i,1));
        QMin=min(QMin,T.VazaoOleo(i,1));
        QMax=max(QMax,T.VazaoOleo(i,1));
        PSucMin=min(PSucMin,T.PressSuccao(i,1));
        PSucMax=max(PSucMax,T.PressSuccao(i,1));
        PChegadaMin=min(PChegadaMin,T.PressChegada(i,1));
        PChegadaMax=max(PChegadaMax,T.PressChegada(i,1));
    end
end
end


