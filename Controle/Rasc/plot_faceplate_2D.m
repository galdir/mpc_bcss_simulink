function plot_faceplate_2D(tag_name,tag_val,tag_sets,Zoom)
    %% HELP plot_faceplate_FIS_control 
    %
    % FunC'C#o para criar uma figura com X quadrantes, um
    % para cada combinaC'C#o de faixas. 
    %
    % Inputs: 
    %   tag_name    - Nome da vaiC!vel textual em vetor coluna (igual ao get_Ranges_BCSS)
    %   tag_val     - Valor de variC!vel instantaneo de operaC'C#o em vetor linha (inserir PVs
    %                 ou MVs que deseja-se plotar no faceplate)
    %   tag_sets    - Valores de mC!ximo e mC-nimo em vetor 2x2 em que tag_val deve estar 
    %                 (Se plot_faceplate_2D for para controle as bandas devem ser inseridas
    %                 como entrada aqui. Se plot_faceplate_2D for para variC!veis manipuladas
    %                 os sets devem ser deixados em zero). 
    %   zoom        - divide as escalas dos eixos para aproximar da origem
    %                 e melhorar visualizaC'C#o
    %
    %% Trata erros 
    % Erro por falta de argumento
    teste=2;
    if nargin==0
        if teste ==1    % Teste faceplate da ER com DELTA_ER
            tag_name=char(["Erro_vazao_oleo_BCSS";"Delta_erro_vazao_oleo_BCSS"]);
            tag_val=[200,2];
            tag_sets=[15,5;...
                 -15,-5];
            Zoom=10;
        elseif teste==2 % Teste faceplate de MVs com limites
            tag_name=char(["Delta_frequencia_BCSS";"Delta_pressao_montante"]);
            tag_val=[0.2,0.1];
            tag_sets=[ ];
            Zoom=2;
        end
           
    end   
    % Erro por dimensC#o diferente de 2
    if size(tag_name,1)~=2 | size(tag_val,2)~=2
        error('DimensC#o dos vetores nC#o C) igual 2');
    end
    
    % Tratar erro por entrada em vetor coluna
    if size(tag_val,1)>1 & size(tag_val,2)==1
        tag_val=tag_val';
    elseif not(size(tag_val,1)>1 & size(tag_val,2)==1 | size(tag_val,1)==1 & size(tag_val,2)>=1  )
        error('DimensC#o dos vetor de valores de Tag errada');       
    end
    
    %% Plota grC!fico  
    % Identifica quantas barras serC#o plotadas e quantos sets existem 
    n_in=2;
    n_set=size(tag_sets,1);
    
    % Identifica quantos quadrantes e linnhas o grC!fico terC! e se 
    % variC!vel estC! no centro do quadrante    
    if n_set==0
        n_quad=4;
        n_lin=2;
    elseif n_set>=1;
        n_quad=(n_set+1)^2;
        n_lin=2*n_set;
    end
    
    % Carrega parametros do grC!fico
    for i=1:n_in
        % busca valores de range minimo e maximo e unidade de engenharia
        tag_label=char(strrep(tag_name(i,:),' ',''));
        [RNGmin,RNGmax,ueng]=get_Ranges_BCSS(char([tag_label]));
        Ueng(i,:)=string(ueng);
        
        % salva retas dos quadrantes
        if n_quad==4  
            % ExtraC'C#o de limite do grC!fico
            Range(i,:)= [RNGmin RNGmax];
        else  
        	Range(i,:)= [RNGmin flip(tag_sets(:,i)') RNGmax];
        end
        
        % Adiciona tC-tulos aos eixos
        tag_label=strrep(tag_label, '_', ' ');
        label(i) = string(sprintf('%s %s',char(tag_label),ueng(:)));       
    end
       
    % Define limites dos eixos
    xlim([Range(1,1)/Zoom Range(1,end)/Zoom]);
    ylim([Range(2,1)/Zoom Range(2,end)/Zoom]);
    
    % Adiciona tC-tulos aos eixos 
    xlabel(char(label(1)));
    ylabel(char(label(2)));
             
    % Define a cor de fundo como cinza claro
    set(gca, 'Color', [0.7 0.7 0.7]);
    
    % Plota ponto de operaC'C#o
    hold on
    X = min(Range(1,end)/Zoom, max(tag_val(1), Range(1,1)/Zoom));
    Y = min(Range(2,end)/Zoom, max(tag_val(2), Range(2,1)/Zoom));
    plot(X,Y,'go','MarkerFaceColor','g','MarkerSize', 6, 'LineWidth', 2);
    plot([0;X],[0;Y], 'g-','LineWidth', 4);
    grid on
    
    % Monta regiC5es dos quadrantes e detecta se variC!vel C) diferente de zero 
    % ou estC! fora regiC#o central  
    if n_quad==4  
        fplt_active=not(all(tag_val'==0));
        % Plota as linhas horizontais e verticais para dividir em 4 quadrantes
        plot([Range(1,1) Range(1,2)], [(Range(2,1) + Range(2,2))/2 (Range(2,1) + Range(2,2))/2], 'k-', 'LineWidth', 1.0); % Linha horizontal no meio
        plot([(Range(1,1) + Range(1,2))/2 (Range(1,1) + Range(1,2))/2], [Range(2,1) Range(2,2)], 'k-', 'LineWidth', 1.0); % Linha vertical no meio
    else
        fplt_active=not(all(Range(:,(n_set+2)/2)<=tag_val' & tag_val'<Range(1,(n_set+2)/2+1)));
        for i=1:n_lin/2 % divide por dois pois sC#o dois eixos
            % Plota as linhas horizontais para dividir em quadrantes
            plot([Range(1,1) Range(1,end)],[Range(2,i+1) Range(2,i+1)], 'k-', 'LineWidth', 1.0); % Linha horizontal inferior
            % Plota as linhas verticais para dividir em quadrantes
            plot([Range(1,i+1) Range(1,i+1)],[Range(2,1) Range(1,end)], 'k-', 'LineWidth', 1.0); % Linha vertical esquerda
        end
    end
    hold off
    
    %Define a cor da moldura (Box) para fplt_edge_color (ativo=>cor amarela/inativo=>verde)
    if fplt_active==1
        fplt_edge_color=[1 1 0];
    else
        fplt_edge_color=[0 1 0];
    end
    rectangle('Position', [Range(1,1)/Zoom Range(2,1)/Zoom (Range(1,end)-Range(1,1))/Zoom (Range(2,end)-Range(2,1))/Zoom],...
              'EdgeColor',fplt_edge_color, 'LineWidth', 2);

    % TC-tulo do grC!fico
    resumo1=char(strrep(tag_name(1,:),' ',''));
    resumo1=char(strrep(resumo1,'_',' '));
    resumo2=char(strrep(tag_name(2,:),' ',''));
    resumo2=char(strrep(resumo2,'_',' ')); 
    texto = sprintf('Faceplate 2D \n %s = %05.2f %s \n %s = %05.2f %s',...
            resumo1,tag_val(1),char(Ueng(1,:)),resumo2,tag_val(2),char(Ueng(2,:)));
    title(texto);
    
