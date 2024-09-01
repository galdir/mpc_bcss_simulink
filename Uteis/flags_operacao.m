function F=flags_operacao(T);
% Retorna F=1/0 com informações por coluna
%      Coluna 1, 1 = Alinhamento das válvulas não associa produção
%      Coluna 2, 1 = MPA habilitado
%      Coluna 3, 1 = Amostragem fora do esperado (aceito apenas Ts = 11 ou 12s)

% ========================================================
% Verifica condições desejadas do alinhamento das válvulas
F1=T.posicao_M1.*T.posicao_W1.*~T.posicao_M2.*~T.posicao_W2.*...
    T.posicao_DHSV.*~T.posicao_XO.*~T.posicao_PXOV.*T.posicao_SUC1.*...
    ~T.alinhamento_manifold_teste.*T.alinhamento_manifold_producao.*...
    T.alinhamento_manifold_producao2;
F1=~F1;     % Inverte para coerência da lógica
%     Verifica=(M1)*(W1)*(~M2)*(~W2)*(DHSV)*(~Xo)*(~PXov)*(PSuc1)*(~Ateste)*(Aprod)*(Aprod2);
% ========================================================
% Verifica MPA habilitado ou não
F2=T.habilita_mpa;
% ========================================================
% Verifica tempos de amostragen na faixa. Extrai a variação em segundos
Tam=size(T,1);
Ta=seconds(T.data_hora(2:Tam,1)-T.data_hora(1:Tam-1,1));
F3=~((Ta==11)|(Ta==12));
F3(Tam)=F3(Tam-1);   % Só para completar a dimensão
% ========================================================
% Compõe F para retornar função
F=[F1  F2  F3];
