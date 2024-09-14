Para executar o projeto Simulink deve-se primeiro ajustar adicionar ao path do Matlab as pastas que contem:
- Tabelas Petrobras
- Funções necessárias para o projeto
- Biblioteca de códigos Casadi
- Modelos ESN e LSTM
- Dados da petrobras em format parquet

Dentro desse repositório as seguintes pastas devem ser adicionadas ao path com os comandos abaixo:
```Matlab
addpath('.\Dados');                                         % Arquivos e tabelas Petrobras
addpath('.\Uteis');                                         % Funcionalidade de utilidade geral consumida por vários
addpath('.\Casadi');                                        % Funções da biblioteca Casadi
addpath('.\Modelos\ESN');                                   % Modelos ESN
addpath('.\Modelos\LSTM\modelos_lstm_9n_casadi');           % Modelos LSTM
```

Os dados da Petrobras em formato parquet são mais volumosos e sigilosos e devem ser mantidos em ambiente controlado.
Então, adicione a pasta que tem esses dados no computador que você está usando ao path do seu Matlab.

Uma vez que os paths foram acionados ao Matlab, deve-se abrir o arquivo CarregaDados.m na pasta ./Controle e executá-lo.
Após isso deve-se abrir o arquivo simulink MPC_JUB27.slx e executar ele.
