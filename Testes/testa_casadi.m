% Importar CasADi
import casadi.*

% Parâmetros do problema
N_c = 4;  % Horizonte de controle
N_p = 10; % Horizonte de predição

% Variáveis de estado e controle
x = SX.sym('x', 2);
u = SX.sym('u');

% Dinâmica do sistema (exemplo: sistema de segunda ordem)
dx = [x(2); -x(1) + u];
f = Function('f', {x, u}, {dx});

% Função de custo em etapa
L = x(1)^2 + x(2)^2 + 0.1*u^2;
L_fun = Function('L', {x, u}, {L});

% Formulação do problema NLP
w = {};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g = {};
lbg = [];
ubg = [];

% Estado inicial
Xk = MX.sym('X0', 2);
w = {w{:}, Xk};
lbw = [lbw; -inf; -inf];
ubw = [ubw; inf; inf];
w0 = [w0; 0; 0];

% Função para calcular limites dependentes do controle
calc_limits = @(u) struct('x1_min', -2 - 0.5*u, 'x1_max', 2 + 0.5*u, ...
                          'x2_min', -1 - 0.25*u, 'x2_max', 1 + 0.25*u);

% Loop sobre o horizonte de predição
for k = 0:N_p-1
    % Variável de controle (apenas para os primeiros N_c passos)
    if k < N_c
        Uk = MX.sym(['U_' num2str(k)]);
        w = {w{:}, Uk};
        lbw = [lbw; -1];  % Limite inferior do controle
        ubw = [ubw; 1];   % Limite superior do controle
        w0 = [w0; 0];
    else
        Uk = w{end-N_c+1};  % Manter o último controle constante
    end

    % Função de custo
    J = J + L_fun(Xk, Uk);

    % Dinâmica do sistema
    Xk_next = Xk + f(Xk, Uk);
    w = {w{:}, Xk_next};
    
    % Calcular limites dependentes do controle
    limits = calc_limits(Uk);
    
    % Aplicar limites dependentes do controle
    lbw = [lbw; limits.x1_min; limits.x2_min];
    ubw = [ubw; limits.x1_max; limits.x2_max];
    w0 = [w0; 0; 0];

    % Restrições adicionais (se necessário)
    % g = {g{:}, Xk_next};
    % lbg = [lbg; ...];
    % ubg = [ubg; ...];

    % Atualizar para o próximo passo
    Xk = Xk_next;
end

% Criar o problema NLP
nlp = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));

% Criar o solver
opts = struct();
opts.ipopt.max_iter = 1000;
opts.ipopt.print_level = 0;
opts.print_time = 0;
solver = nlpsol('solver', 'ipopt', nlp, opts);

% Resolver o NLP
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg);

% Extrair a solução
w_opt = full(sol.x);

% Mostrar resultados
disp('Solução ótima:');
disp(['Estados: ' num2str(w_opt(1:2:end)')]);
disp(['Controles: ' num2str(w_opt(3:3:3*N_c)')]);

% Visualizar os limites dependentes do controle
figure;
t = 0:N_p-1;
controls = [w_opt(3:3:3*N_c)' w_opt(end-1)];
for i = 1:2
    subplot(2,1,i);
    hold on;
    plot(t, w_opt(i:2:end), 'b-', 'LineWidth', 2);
    limits = arrayfun(@(u) calc_limits(u), controls);
    plot(t, [limits.x1_min], 'r--', t, [limits.x1_max], 'r--');
    title(['Estado x' num2str(i)]);
    xlabel('Passo de tempo');
    ylabel(['x' num2str(i)]);
    legend('Estado', 'Limite inferior', 'Limite superior');
    hold off;
end