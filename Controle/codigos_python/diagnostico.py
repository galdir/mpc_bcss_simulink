import os
import sys

def diagnosticar():
    print("=== Diagnóstico de Ambiente ===")
    print("\n1. Diretório atual:")
    print(os.getcwd())
    
    print("\n2. PYTHONPATH:")
    for path in sys.path:
        print(path)
    
    print("\n3. Estrutura de diretórios:")
    for root, dirs, files in os.walk('.'):
        print(f"\nDiretório: {root}")
        print("Arquivos:", files)
        
    print("\n4. Tentando importar módulos:")
    try:
        import casadi
        print("✓ CasADi importado com sucesso")
    except ImportError as e:
        print("✗ Erro ao importar CasADi:", e)
    
    try:
        import numpy
        print("✓ NumPy importado com sucesso")
    except ImportError as e:
        print("✗ Erro ao importar NumPy:", e)
    
    print("\n5. Verificando arquivo de teste:")
    teste_path = os.path.join('testes', 'teste_estima_vazao.py')
    if os.path.exists(teste_path):
        print(f"✓ Arquivo {teste_path} encontrado")
        with open(teste_path, 'r') as f:
            print("\nPrimeiras linhas do arquivo:")
            print(''.join(f.readlines()[:10]))
    else:
        print(f"✗ Arquivo {teste_path} não encontrado")

if __name__ == "__main__":
    diagnosticar()