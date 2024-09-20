function NovaT=RepSimulador(SalvaTabela, PassoF,PassoP);
%  Função para gerar nova tabela (com maior resoluçlão), com base na tabela original do Simulador
%  
% Permite salvar a nova Tabela num arquivo XLSX ou apenas consumir para
%  usos específicos (por exemplo, para traçar as curvas de isovazão)
% 
% Os parâmetros de GridF e GridP são os grids de Frequência e Pressão com os quais construiremos a nova tabela
%
% Já que sabemos reroduzir parcialmente as contas e fazemos interpolações,
% podemos gerar dados REPRODUZINDO parcialmente as tabelas geradas pelo
% simulador Petrobras, com diferentes grids para a Frequencia e para a Pressão de Chegada
%
% OBS: Necessário ajustar o path para visualizar as tabelas de dados da Petrobras

if nargin<1                    % Não foi passar parâmetro pelo usuário
    SalvaTabela=1;       % Por padrão, salva a tabela em um novo arquivo XLSX.
    PassoF=0.5;                % Por padrão, usa grid de 0,5Hz como passo para montar o Grid de Frequência (do Min ao Max)  
    PassoP=0.5;                % Por padrão, usa grid de 0,5 Kgf/cm2 como passo para montar o Grid de Pressão (do Min ao Max) 
end

% Carrega tabela com dados do simulador (já converte os nomes no formato adotado)
T=LeConverteNomes('DoSimulador.xlsx');

%======================================================
% Limites para as interpolações 
FMin=min(T.FreqBCSS);       % Assume o valor minimo de Frequencia existente na tabela do simulador
FMax=max(T.FreqBCSS);    % Assume o valor máximo de Frequencia existente na tabela do simulador

PMin=min(T.PressChegada);       % Assume o valor minimo de PChegada existente na tabela do simulador
PMax=max(T.PressChegada);    % Assume o valor máximo de PChegada existente na tabela do simulador

%======================================================
% Carrega 1 registro da tabela atual apenas para ter a estrutura e inserir registros
NovaT=T(1,:);

i=1; % contador de registros
for Freq=FMin:PassoF:FMax
    for Pchegada=PMin:PassoP:PMax
        NewReg=Interpola(Freq,Pchegada,T);
        NovaT(i,:)=NewReg;
        i=i+1;        % Incrementa contador de registro
    end
end

%==============================
% Se quiser salvar a nova tabela
if SalvaTabela
    writetable(NovaT,'DaNovaTabela.xlsx');
end