import casadi as ca

def busca_limites_matriz_casadi(matriz, valor_procurado):
    matriz = ca.MX(matriz)
    
    # Número de linhas e colunas
    n = matriz.size1()      # Número de linhas na coluna
    n2 = matriz.size2()     # Número de colunas
    
    # Ajuste do valor procurado
    valor_procurado = ca.ceil(valor_procurado * 10) / 10
    valor_procurado = ca.if_else(valor_procurado < 40, 40, valor_procurado)
    valor_procurado = ca.if_else(valor_procurado > 60, 60, valor_procurado)
    
    # Inicialização das variáveis
    contador = ca.MX.zeros(1)
    lim_max = ca.MX.zeros(1, n2)
    lim_min = ca.MX.zeros(1, n2)
    
    # Iteração pelas linhas
    for i in range(n):
        condicao = ca.logic_and(matriz[i, 0] == valor_procurado, contador < 2)
        
        # Atualização dos limites
        lim_max = ca.if_else(ca.logic_and(condicao, contador == 0), 
                         matriz[i, :], lim_max)
        lim_min = ca.if_else(ca.logic_and(condicao, contador == 1), 
                         matriz[i, :], lim_min)
        
        # Atualização do contador
        contador = contador + ca.if_else(condicao, 1, 0)
    
    # Retorna os limites excluindo a primeira coluna
    return ca.vertcat(lim_max[:, 1:], lim_min[:, 1:])


# Exemplo de uso
if __name__ == "__main__":
    from pathlib import Path
    import pandas as pd
    import casadi as ca
    
    freq_sym = ca.MX.sym('freq_sym',1)        # Cria variável simbólica para a Frequencia

    arquivo_limites_integrados = 'TabelaLimitesDinamicos.xlsx' # Tabela completa com pré-cálculos dos limites dinâmicos em função da frequência
    caminho_tabelas = Path("./Tabelas")
    df_limites_integrados = pd.read_excel(
        caminho_tabelas / arquivo_limites_integrados)
    matriz_limites_integrados = df_limites_integrados.values

    busca_limites_matriz_casadi_sym = busca_limites_matriz_casadi(matriz_limites_integrados, freq_sym)
    busca_limites = ca.Function('busca_limites', {freq_sym}, {busca_limites_matriz_casadi_sym})