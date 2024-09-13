function plot_faceplate_1D_bar(tag_name,tag_val,tag_sets)
    %% HELP plot_faceplate_FIS_control 
    %
    % FunC'C#o para criar uma figura com grC!fico que mostra os valores PV em
    % barra, com os ranges e os sets, como um faceplate de SDCD/SCADA.
    %
    % Inputs: 
    %   tag_name    - Nome da variC!vel textual em vetor coluna (igual ao get_Ranges_BCSS)
    %   tag_val     - Valor de variC!vel instantaneo de operaC'C#o em vetor linha (inserir PV
    %                 ou MV que deseja-se plotar no faceplate)
    %   tag_sets    - Valores de sets mC!ximo e mC-nimo em vetor 2xX em que tag_val deve estar 
    %                 (Se plot_faceplate_bar for para controle o SP 
    %                 deve ser inserido como entrada aqui. Os limites 
    %                 max e min serC#o colapsados no valor de SP). 
    %
    %% Trata erro por falta de argumento
    teste=2;
    if nargin ==0
        if teste ==1    % Teste faceplate da PV com SP
            tag_name=char(["vazao_oleo_BCSS"]);
            tag_val=[300];
            tag_sets=[400];
        elseif teste ==2% Teste faceplate de MVs com limites
            tag_name=char(["pressao_montante_eng";"frequencia_BCSS"]);
            tag_val=[35,55];
            tag_sets=[40,60;...
                  20,40];
        end
    end
    %% Plota grC!fico
    % identifica quantas barras serC#o plotadas e quantos sets existem 
    n_set=size(tag_sets,1);
    n_in=size(tag_val,2);
    
    % identifica se o grC!fico C) de controle
    if n_set<2
       tag_sets=[tag_sets;...
              tag_sets];
    end   
    
    % Plota cada variC!vel em seu prC3prio as barras
    for i=1:n_in
        % separa cada barra em seu subplot com seus parametros
        subplot(1,n_in,i)
        
        % plota a barra com o valor da variC!vel
        bar_lar=6; %largura da barra
        barra = bar(tag_val(i), 'FaceColor', 'green', 'EdgeColor', 'black',...
                   'LineWidth', 1,'BarWidth', bar_lar);
               
        % busca valores de range minimo e maximo e unidade de engenharia
        tag=char(strrep(tag_name(i,:),' ', ''));
        [RNGmin,RNGmax,ueng]=get_Ranges_BCSS(char([tag]));
        
        % ajusta barra com os ranges da variC!vel respectiva
        ylim([RNGmin RNGmax]);
        
        % desabilita valores do eixo X
        xticks([]);
            
        % identifica posiC'C#o do topo da barra e plota valor
        xtips = barra(1).XEndPoints;
        ytips = barra(1).YEndPoints;
        labels = sprintf('%05.2f %s',barra(1).YData,ueng);
        text(xtips,ytips,labels,'HorizontalAlignment','center',...
                                'VerticalAlignment','top')
        % posiC'C#o dos sets H e L no grC!fico
        line_comp=bar_lar/2;
        
        % desenha sets H e L
        line([1-line_comp 1+line_comp], [tag_sets(1,i) tag_sets(1,i)], 'Color', 'r', 'LineWidth', 1.5);
        line([1-line_comp 1+line_comp], [tag_sets(2,i) tag_sets(2,i)], 'Color', 'r', 'LineWidth', 1.5);
        
        % desenha o triC"ngulos nos sets H e L
        tri_larg=bar_lar/10 ;    %tamanho do lado do tringulo
        tri_altu_escale=RNGmax/100;
        triH_x = [1-line_comp 1-line_comp+tri_larg 1-line_comp];
        triH_y = [tag_sets(1,i)-sqrt((4*tri_larg^2)/3)*tri_altu_escale tag_sets(1,i) tag_sets(1,i)+sqrt((4*tri_larg^2)/3)*tri_altu_escale];
        patch(triH_x, triH_y, 'r');  
        
        triL_x = [1-line_comp 1-line_comp+tri_larg 1-line_comp];
        triL_y = [tag_sets(2,i)-sqrt((4*tri_larg^2)/3)*tri_altu_escale tag_sets(2,i) tag_sets(2,i)+sqrt((4*tri_larg^2)/3)*tri_altu_escale];
        patch(triL_x, triL_y, 'r'); 
        
        % adiciona o valor numC)rico de H e L ao lado direito do triC"ngulo
        text(1.2+line_comp, min(tag_sets(1,i)+2,RNGmax-2), num2str(tag_sets(1,i)), 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
        text(1.2+line_comp, max(tag_sets(2,i)-2,RNGmin-2), num2str(tag_sets(2,i)), 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
        
        % adiciona o valor numC)rico de H e L ao lado direito do triC"ngulo e 
        % legenda de linhas de set
        if n_set<2
            text(0.25-line_comp, max(min(tag_sets(1,i)+2,RNGmax-2),RNGmin-2), 'SP', 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
            text(1.2+line_comp, min(tag_sets(1,i)+2,RNGmax-2), num2str(tag_sets(1,i)), 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
        else
            text(0.25-line_comp, min(tag_sets(1,i)+2,RNGmax-2), 'Max', 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
            text(0.25-line_comp, max(tag_sets(2,i)-2,RNGmin-2), 'Min', 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
            text(1.2+line_comp, min(tag_sets(1,i)+2,RNGmax-2), num2str(tag_sets(1,i)), 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
            text(1.2+line_comp, max(tag_sets(2,i)-2,RNGmin-2), num2str(tag_sets(2,i)), 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');

        end  
        
        % Define a cor de fundo como cinza claro
        set(gca, 'Color', [0.7 0.7 0.7]);
        
        % TC-tulo do grC!fico
        tag=strrep(tag, '_', ' ');
        texto = sprintf('Faceplate %s %s',char(tag),ueng);
        title(texto); 
        hold on
        
    end
    


   



    
    
    
    
    
    
    
    
    
    
    
    
    