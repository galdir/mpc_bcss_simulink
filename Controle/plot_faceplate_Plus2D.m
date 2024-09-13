function plot_faceplate_Plus2D(tag_namein,tag_valin,tag_setsin,tag_hab)
    %% HELP plot_faceplate_FIS_control 
    %
    % FunC'C#o para criar uma figura com X quadrantes, um
    % para cada combinaC'C#o de faixas. 
    %
    % Inputs: 
    %   tag_namein  - Nome da variC!vel textual em vetor coluna (igual ao get_Ranges_BCSS)
    %   tag_valin   - Valor de variC!vel instantaneo de operaC'C#o em vetor linha (inserir PVs
    %                 ou MVs que deseja-se plotar no faceplate)
    %   tag_setsin  - Valores de mC!ximo e mC-nimo em vetor 2x2 em que tag_val deve estar 
    %                 (Se plot_faceplate_Plus2D for para controle as bandas devem ser inseridas
    %                 como entrada aqui. Se plot_faceplate_Plus2D for para variC!veis manipuladas
    %                 os sets devem ser deixados em zero). 
    %   tag_hab     - Vetor com informaC'C#o de quais variC!veis devem ser representadas no grC!fico
    %                 (0->variC!vel inibida / 1->variC!vel visC-vel)
    %
    %% Trata erros    
    % Erro por falta de argumento
    teste=2; %ver teste 3 com 10 variC!veis
    if nargin==0
        if teste ==1    % Teste faceplate longevidade
            tag_namein=char(["vazao_oleo_BCSS";"Delta_corrente_torque_BCSS";"Oscilacao_corrente_torque_BCSS";...
                             "vibracao_BCSS";"temperatura_succao_BCSS"]);
            tag_valin =[300,0.6,  3,1, 250];
            tag_setsin=[500, 25, 25,2,180;...
                        350,-25,-25,0,0];
            tag_hab = [1,1,1,1,1];
           
       elseif teste==2 % Teste faceplate anttitrip
            tag_namein=char(["pressao_succao_BCSS";"pressao_diferencial_BCSS";"pressao_descarga_BCSS";...
                             "pressao_chegada";"corrente_total_BCSS";"corrente_torque_BCSS";...
                             "temperatura_motor_BCSS";"temperatura_succao_BCSS";"vibracao_BCSS";"temperatura_chegada";"vazao_oleo_BCSS"]);
            tag_valin =[80,110,200,50,184, 98,110,250,1,50,300];
            tag_setsin=[84,149,214,65,183,138,141,180,2,70,500;...
                        76, 52,123,27, 20,104,22 ,0  ,0,0 ,350];
            tag_hab = [1,1,1,1,1,1,1,0,0,0,0]; 

        elseif teste==3 % Teste faceplate MPO restriC'C5es
            tag_namein=char(["pressao_succao_BCSS";"pressao_diferencial_BCSS";"pressao_descarga_BCSS";...
                             "pressao_chegada";"corrente_total_BCSS";"corrente_torque_BCSS";...
                             "temperatura_motor_BCSS";"temperatura_succao_BCSS";"vibracao_BCSS";"temperatura_chegada";"vazao_oleo_BCSS"]);
            tag_valin =[80,110,200,50,184, 98,110,250,1,50,300];
            tag_setsin=[84,149,214,65,183,138,141,180,2,70,500;...
                        76, 52,123,27, 20,104,22 ,0  ,0,0 ,350];
            tag_hab = [1,1,1,1,1,1,1,1,1,1,1];     

        end
    end   
    
    % Erro por dimensC#o diferente de 2
    if size(tag_namein,1)~=size(tag_valin,2) | size(tag_namein,1)~=size(tag_setsin,2)
        error('DimensC#o dos vetores C) diferente');
    end
    
    % Tratar erro por entrada em vetor coluna
    if size(tag_valin,1)>1 & size(tag_valin,2)==1
        tag_valin=tag_valin';
    elseif not(size(tag_valin,1)>1 & size(tag_valin,2)==1 | size(tag_valin,1)==1 & size(tag_valin,2)>=1  )
        error('DimensC#o dos vetor de valores de Tag errada');       
    end
    
    % Tratar erro por entrada errada no vetor tag_hab
    if ~isequal(size(tag_valin), size(tag_hab))
        error('As dimensC5es dos vetores tag_val e tag_hab devem ser iguais.');
    end
     
    %% Plota grC!fico  
    % Filtra as variC!veis que estC#o habilitas pelo tag_hab
    tag_name= tag_namein(tag_hab == 1,:);   
    tag_val = tag_valin(1,tag_hab == 1);
    tag_sets(1,:)= tag_setsin(1,tag_hab == 1);
    tag_sets(2,:)= tag_setsin(2,tag_hab == 1);
    
    % Identifica quantas barras serC#o plotadas e quantos sets existem 
    n_in=size(tag_val,2);
    n_set=size(tag_sets,1);
            
    % Carrega parametros do grC!fico
    for i=1:n_in
        % Busca valores de range minimo e maximo e unidade de engenharia
        tag_aux=char(strrep(tag_name(i,:),' ',''));
        [RNGmin,RNGmax,ueng,tag_curto]=get_Ranges_BCSS(char([tag_aux]));
        Ueng(i,:)=string(ueng);
        
        % Monta matriz de nomes das tags que aparecem no grC!fico
        tag_label(i,:)=cellstr(tag_curto);
        
        % Cria vetor linha com todas informaC'C5es por variC!vel
        Range(i,:)= [RNGmin flip(tag_sets(:,i)') RNGmax];
        
        % Cria vetores coluna com tipos de informaC'C#o por todas variC!veis
        Rng_max(:,i)= Range(i,4);
        SETs_H(:,i)= Range(i,3);
        SETs_L(:,i)= Range(i,2);
        Rng_min(:,i)= Range(i,1);
        
        %Identifica qual alarme estC! ativo
        SEL_H_active(i)=(tag_val(i)>=SETs_H(i));
        SEL_L_active(i)=(tag_val(i)<=SETs_L(i));
        SEL_N_active(i)=(tag_val(i)>=SETs_H(i) | tag_val(i)<=SETs_L(i));
        
        % Estado do texto dos alarmes que aparecerC#o na tabela resumo
        if SEL_L_active(i)
            Alarm_txt_sts(i,1:3)=[1, 0, 0];
        else
            Alarm_txt_sts(i,1:3)=[1, 1, 1];
        end
        if SEL_N_active(i)
            Alarm_txt_sts(i,4:6)=[1, 1, 1];
        else
            Alarm_txt_sts(i,4:6)=[0, 1, 0];
        end
        if SEL_H_active(i)
            Alarm_txt_sts(i,7:9)=[1, 0, 0];
        else
            Alarm_txt_sts(i,7:9)=[1, 1, 1];
        end
    end
       
    % CriaC'C#o das regiC'C5es
    axes_shaded_limits = {...             % [min axes limits; max axes limits]
                    [Rng_min; SETs_L],... % [regiC#o de alarme L]
                    [SETs_L; SETs_H],...  % [regiC#o de alarme N]
                    [SETs_H; Rng_max]};   % [regiC#o de alarme H]

    % Spider plot
    plot_values=0;  %  plot_values=0 sem valores no grC!fico, plot_values=1 com valores visC-veis
    data_curves=[tag_val;Rng_min;Rng_max;SETs_L;SETs_H];
    if plot_values==1
        splot=spider_plot(data_curves,...
            'AxesLabels', tag_label,... 
            'AxesLimits',[Rng_min;Rng_max],...
            'AxesShaded', 'on',...
            'AxesShadedLimits', axes_shaded_limits,...
            'AxesShadedColor', {'r','g','r'},...
            'AxesInterval', 4,...
            'AxesDisplay', 'data',...
            'AxesFontColor', [0,0,0;0,0,0;0,0,0;0,0,0;0,0,0],...
            'AxesShadedTransparency', 0.2,...
            'AxesFontSize', 8,...
            'Color', [0,0,0;0,0,0;0,0,0;1,0,0;1,0,0],...
            'LineTransparency', 0,...
            'Marker',['o';'.';'.';'.';'.'],...
            'MarkerSize', 40,...
            'LabelFontSize', 8,...
            'BackgroundColor' , [0.97 0.97 0.97]); 
    else
        splot=spider_plot(data_curves(1,:),...
            'AxesLabels', tag_label,...
            'AxesLimits',[Rng_min;Rng_max],...
            'AxesShaded', 'on',...
            'AxesShadedLimits', axes_shaded_limits,...
            'AxesShadedColor', {'r','g','r'},...    
            'AxesInterval', 4,...
            'AxesDisplay', 'data',...
            'AxesFontColor', [0,0,0],...
            'AxesShadedTransparency', 0.2,...
            'AxesFontSize', 8,...
            'Color', [0,0,0],...
            'LineTransparency', 0,...
            'Marker',['o'],...
            'MarkerSize', 40,...
            'LabelFontSize', 8,...
            'BackgroundColor' , [0.97 0.97 0.97]);
    end
    
    % Ajusta zoom para caber tabela abaixo do grC!fico
    Zoom=1.4;
    
    % TC-tulo do grC!fico       
    title('Faceplate +2D');    
    
    %Destaca fundo do grC!fico com cor respectiva a status de alarme 
    %(ativo=>cor amarela/inativo=>verde)
    if SEL_N_active==1
        alarmG_color = [0 0 0];
    else
        alarmG_color = [0 1 0];
    end    
  
    % Identifica posiC'C#o do plot, da zoom e sobe para caber resumo de variC!veis       
    pos_int = get(gca, 'Position');
    pos_int(1)=pos_int(1)+(pos_int(3)-pos_int(3)/Zoom)/2; % anda para direita com metade da folga dada pelo zoom para centralizar    
    pos_int(2)=pos_int(2)+(pos_int(4)-pos_int(4)/Zoom);   % sobe grC!fico com toda folga dada pelo zoom
    set(gca, 'Position', [pos_int(1),pos_int(2),pos_int(3)/Zoom,pos_int(4)/Zoom])
    
    % Identifica posiC'C#o da figura
    set(gcf, 'Units', 'normalized')
    pos_ext = get(gcf, 'Position');  
        
    % Cria fundo para tabela com o resumo dos valores de tag_val e alarmes
    C0_tab=0.05;                    % PosiC'C#o X de borda esquerda e inferior da tabela
    L0_tab=0.03;                    % PosiC'C#o Y de borda esquerda e inferior da tabela
    TamX_tab=1-2*C0_tab;            %Inicio em C0 e tamanho simC)trico com folga de C0 nos dois lados
    TamY_tab=1-pos_ext(4)-0.1;      %Tamanho atC) a altura da figura, com folga de lagenda
    annotation('rectangle', [C0_tab,L0_tab,TamX_tab,TamY_tab],'FaceColor',[0.95, 0.95, 0.95],'Color', 'k', 'LineWidth', 1.5); 

    % Definir a cor da moldura geral 
    C0_mold=C0_tab;                     %PosiC'C#o X da moldura C) a mesma da tabela
    L0_mold=TamY_tab+L0_tab+0.005;      %PosiC'C#o Y da moldura C) onde acaba tabela mais folga           
    TamX_mold=TamX_tab;                 %Inicio em C0 e tamanho simC)trico com folga de C0 nos dois lados
    TamY_mold=1-TamY_tab-L0_tab-L0_tab; %Tamanho da moldura C) altura com folga do tC-tulo
    annotation('rectangle', [C0_mold,L0_mold,TamX_mold,TamY_mold], 'Color', alarmG_color, 'LineWidth', 2);         
    
    % Cria tabela com o resumo dos valores das entradas (PVs) e alarmes
    Res_table_values(1,:,1) = string({'VariC!vel','SET L','Tag Valor','SET H','Unidade'});
    for i=1:6
        tag_aux=char(strrep(tag_name(i,1:20),' ',''));
        tag_aux=char(strrep(tag_aux,'_',' '));
        tags =string(tag_aux);
        text1=string(SETs_L(i));   
        resum=string(sprintf('%06.3f',tag_val(i)));
        text2=string(SETs_H(i)); 
        text3=string(Ueng(i,:));
        Res_table_values(i+1,:,1) = [tags,text1,resum,text2,text3]; 
    end   
    if n_in>6 & n_in<=12
        Res_table_values(1,:,2) = string({'VariC!vel','SET L','Tag Valor','SET H','Unidade'});
        for i=7:n_in
            tag_aux=char(strrep(tag_name(i,1:20),' ',''));
            tag_aux=char(strrep(tag_aux,'_',' '));
            tags =string(tag_aux);
            text1=string(SETs_L(i));   
            resum=string(sprintf('%06.3f',tag_val(i)));
            text2=string(SETs_H(i)); 
            text3=string(Ueng(i,:));
            Res_table_values(i-5,:,2) = [tags,text1,resum,text2,text3];
        end           
    elseif n_in>12
        error('DimensC#o dos vetor de valores muito grande para ser mostrada no espaC'o da tabela');       
    end  
    
    % Insere valores da primeira coluna no espaC'o da tabela 
    L0_tab=L0_tab+(5/6)*TamY_tab-0.005;
    C0_tab=C0_tab+0.0075;
%     dist_lin=TamY_tab/6-0.0025;
    dist_lin=TamY_tab/7-0.0025;
    dist_col=0.06;
    dist_col_1=0.2;   
    
    % Imprime primeira linha da tabela com atC) 6 variC!veis (primeira coluna
    % C) maior por causa dos nomes das variC!veis
    annotation('textbox', [C0_tab,L0_tab,dist_col_1,dist_lin], 'String', Res_table_values(1,1,1), ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'EdgeColor', 'none', 'FontSize', 7,'BackgroundColor',[0.9, 0.9, 0.9]);
    for i=2:5
        annotation('textbox', [C0_tab+dist_col_1+(i-2)*dist_col,L0_tab,dist_col,dist_lin], 'String', Res_table_values(1,i,1), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'EdgeColor', 'none', 'FontSize', 7,'BackgroundColor',[0.9, 0.9, 0.9]);
    end
    % Imprime primeira linha da 2a. tabela de variavel 6 a 12 
    L0_tab2=L0_tab;
    C0_tab2=C0_tab+dist_col_1+4*dist_col;
    if n_in>6 & n_in<=12 
        annotation('textbox', [C0_tab2,L0_tab2,dist_col_1,dist_lin], 'String', Res_table_values(1,1,2), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'EdgeColor', 'none', 'FontSize', 7,'BackgroundColor',[0.9, 0.9, 0.9]);
        for i=2:5
            annotation('textbox', [C0_tab2+dist_col_1+(i-2)*dist_col,L0_tab2,dist_col,dist_lin], 'String', Res_table_values(1,i,2), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                'EdgeColor', 'none', 'FontSize', 7,'BackgroundColor',[0.9, 0.9, 0.9]);
        end
    end

    % Imprime demais linhas da tabela com atC) 6 variC!veis (primeira coluna
    L0_tab=L0_tab-dist_lin;         % abate a linha do cabeC'C!rio
    C0_tab=C0_tab;
    L0_tab2=L0_tab2-dist_lin;       % abate a linha do cabeC'C!rio
    C0_tab2=C0_tab2;
    for j=1:6
        annotation('textbox', [C0_tab,L0_tab-(j-1)*dist_lin,dist_col_1,dist_lin], 'String', Res_table_values(j+1,1,1), ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', ...
            'EdgeColor', 'none', 'FontSize', 7,'BackgroundColor',[1, 1, 1]);
        for i=2:4
            annotation('textbox', [C0_tab+dist_col_1+(i-2)*dist_col,L0_tab-(j-1)*dist_lin,dist_col,dist_lin], 'String', Res_table_values(j+1,i,1), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FaceAlpha',0.15,...
                'EdgeColor', 'none', 'FontSize', 7,'BackgroundColor',Alarm_txt_sts(j,(i-2)*3+1:(i-2)*3+3));
        end
        i=i+1;
        annotation('textbox', [C0_tab+dist_col_1+(i-2)*dist_col,L0_tab-(j-1)*dist_lin,dist_col,dist_lin], 'String', Res_table_values(j+1,i,1), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'EdgeColor', 'none', 'FontSize', 7,'BackgroundColor',[1, 1, 1]);
    end
    
    % Imprime demais linhas da 2a. tabela de variavel 6 a 12
    if n_in>6 & n_in<=12 
        L0_tab2=L0_tab;
        C0_tab2=C0_tab+dist_col_1+4*dist_col;
        for j=1:n_in-6
            annotation('textbox', [C0_tab2,L0_tab2-(j-1)*dist_lin,dist_col_1,dist_lin], 'String', Res_table_values(j+1,1,2), ...
                'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', ...
                'EdgeColor', 'none', 'FontSize', 7,'BackgroundColor',[1, 1, 1]);
            for i=2:4
                annotation('textbox', [C0_tab2+dist_col_1+(i-2)*dist_col,L0_tab2-(j-1)*dist_lin,dist_col,dist_lin], 'String', Res_table_values(j+1,i,2), ...
                    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FaceAlpha',0.15,...
                    'EdgeColor', 'none', 'FontSize', 7,'BackgroundColor',Alarm_txt_sts(j,(i-2)*3+1:(i-2)*3+3));
            end
            i=i+1;
            annotation('textbox', [C0_tab2+dist_col_1+(i-2)*dist_col,L0_tab2-(j-1)*dist_lin,dist_col,dist_lin], 'String', Res_table_values(j+1,i,2), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                'EdgeColor', 'none', 'FontSize', 7,'BackgroundColor',[1, 1, 1]);
        end
    end
  

    
          
                
                
                
                
                
                
      
