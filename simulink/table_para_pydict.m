function pyDict = table_para_pydict(tbl)
pyDict = py.builtins.dict();
varNames = tbl.Properties.VariableNames;
for i = 1:length(varNames)
    varName = varNames{i};
    if length(tbl.(varName)) > 1
        pyDict{varName} = py.builtins.list(tbl.(varName));
    else
        pyDict{varName} = tbl.(varName);
    end
end
end