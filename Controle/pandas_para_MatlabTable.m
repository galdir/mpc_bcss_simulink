function tab = pandas_para_MatlabTable(df)
    tab = table;
    records = cell(df.values.tolist);
    if(py.isinstance(df, py.type(py.pandas.DataFrame)))
        for k = 1:length(records)
            row = recordDecoding(records{k});
            tab = [tab; row];
        end
        headerCells = df.columns.to_list;
        for k = 1:length(headerCells)
            header(k) = headerCells{k}.string;
        end
        
        % Check if there are columns with the same name
        if(length(unique(header)) == length(header))
            tab.Properties.VariableNames = header;
        else
            for name = unique(header)
                sameNameCol = find(name == header);
                if(length(sameNameCol) == 1)
                    continue;
                end
                tab = [tab(:,1:sameNameCol(1)-1), table(tab(:, sameNameCol).Variables, 'VariableName', name), tab(:,sameNameCol(1):end)];
                tab(:, sameNameCol+1) = [];
                header(sameNameCol(2:end)) = [];
            end
            tab.Properties.VariableNames = header;
        end
    elseif(py.isinstance(df, py.type(py.pandas.Series)))
        for l = 1:length(records)
            [value, valType] = fieldDecoding(records{l});
            row = table(value, 'VariableNames', string(df.name.char));
            tab = [tab; row];
        end
        tab.Properties.RowNames = string(df.index.values.tolist);
    end
end
function row = recordDecoding(record)
    fields = cell(record);
    row = table;  
    for l = 1:length(fields)
        [value, valType] = fieldDecoding(fields{l});
        tableCell = table(value, 'VariableNames', ["Var" + l]);
        row = [row, tableCell];
    end
end
function [value, type] = fieldDecoding(field)
    if(py.isinstance(field, py.type(py.bool)))
        value = logical(field);
        type = 'logical';
    elseif(py.isinstance(field, py.type(py.int)))
        value = field.int64;
        type = 'int64';
    elseif(py.isinstance(field, py.type(py.str)))
        value = string(field);
        type = 'string';
    elseif(py.isinstance(field, py.type(py.float)))
        value = double(field);
        type = 'double';
    elseif(py.isinstance(field, py.type(py.complex)))
        value = complex(field);
        type = 'complex';
    elseif(py.isinstance(field, py.type(py.list)))
        value = cell(field);
        type = 'cell';
    else
        value = field;
    end
end


%% versoes anteriores
function matlab_table = pandas_para_MatlabTable_0_1(pandas_df)
% Converte um DataFrame do pandas para uma tabela do MATLAB

% Verifica se o input é um DataFrame do pandas
if ~isa(pandas_df, 'py.pandas.core.frame.DataFrame')
    error('Input deve ser um DataFrame do pandas');
end

% Obtém os dados do DataFrame como uma matriz numpy
py_data = pandas_df.to_numpy();

% Converte a matriz numpy para uma matriz MATLAB
matlab_data = double(py_data);

% Obtém os nomes das colunas
column_names = cellfun(@char, cell(pandas_df.columns.tolist()), 'UniformOutput', false);

% Cria a tabela do MATLAB
matlab_table = array2table(matlab_data, 'VariableNames', column_names);

end


function matlab_table = pandas_to_matlab_table_0_2(pandas_df)
    % Converte um DataFrame do pandas para uma tabela do MATLAB
    
    % Verifica se o input é um DataFrame do pandas
    if ~isa(pandas_df, 'py.pandas.core.frame.DataFrame')
        error('Input deve ser um DataFrame do pandas');
    end
    
    % Obtém os nomes das colunas
    column_names = cellfun(@char, cell(pandas_df.columns.tolist()), 'UniformOutput', false);
    
    % Inicializa um contêiner para armazenar os dados das colunas
    column_data = cell(1, length(column_names));
    
    % Processa cada coluna individualmente
    for i = 1:length(column_names)
        col_name = column_names{i};
        col_type = char(pandas_df.dtypes.iloc(i-1));
        col_values = pandas_df{col_name}.values;
        
        switch col_type
            case 'int64'
                column_data{i} = int64(col_values);
            case 'float64'
                column_data{i} = double(col_values);
            case 'bool'
                column_data{i} = logical(col_values);
            case 'datetime64[ns]'
                column_data{i} = datetime(col_values, 'ConvertFrom', 'posixtime', 'TicksPerSecond', 1e9);
            case 'object'
                % Assume que objetos são strings
                column_data{i} = cellstr(py.numpy.char.decode(col_values.astype('str')));
            otherwise
                warning('Tipo de dados não reconhecido para a coluna %s: %s. Convertendo para célula.', col_name, col_type);
                column_data{i} = cell(col_values);
        end
    end
    
    % Cria a tabela do MATLAB
    matlab_table = table(column_data{:}, 'VariableNames', column_names);
end


function matlab_table = pandas_to_matlab_table_0_3(pandas_df)
    % Converte um DataFrame do pandas para uma tabela do MATLAB
    
    % Verifica se o input é um DataFrame do pandas
    if ~isa(pandas_df, 'py.pandas.core.frame.DataFrame')
        error('Input deve ser um DataFrame do pandas');
    end
    
    % Obtém os dados do DataFrame como uma matriz numpy
    py_data = pandas_df.to_numpy();
    
    % Converte a matriz numpy para uma matriz MATLAB
    matlab_data = double(py_data);
    
    % Obtém os nomes das colunas
    column_names = cellfun(@char, cell(pandas_df.columns.tolist()), 'UniformOutput', false);
    
    % Cria a tabela do MATLAB
    matlab_table = array2table(matlab_data, 'VariableNames', column_names);
    
end