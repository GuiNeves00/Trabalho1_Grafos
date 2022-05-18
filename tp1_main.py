from pickle import TRUE
import sys
sys.path.append("..")
import tp1_grafo as grafoT1
import time

def eh_ponderado(Grafo):
    ponderado = False
    for i in range(len(Grafo.lista_adj)):
        for j in range(len(Grafo.lista_adj[i])):
            if Grafo.lista_adj[i][j][1] != 1:
                return False
            elif Grafo.lista_adj[i][j][1] == 1:
                ponderado = True
    return ponderado

def aresta_negativa(Grafo):
    for i in range(len(Grafo.lista_adj)):
        for j in range(len(Grafo.lista_adj[i])):
            if Grafo.lista_adj[i][j][1] < 0:
                return True
    
    return False

def obtem_caminho(s, t, pred):
    caminho = [t]
    x = t

    while x != s: # Linhas para recuperar o caminho de um vertice ao outro.
        x = pred[x]
        caminho.append(x)
    
    caminho.reverse() # Linha para inverter o conteudo de "caminho" para recuperar o caminho corretamente.

    return caminho

def resultados(origem, destino, caminho, custo):
    print("Origem: ", origem)
    print("Destino: ", destino)
    print("Caminho: ", caminho)
    print("Custo: ", custo)

grafo = grafoT1.Grafo()
nome_arq = input("Digite o nome do arquivo:")
grafo.ler_arquivo(nome_arq)


if eh_ponderado(grafo) == True:
    print("p")
    origem = int(input("Digite a ORIGEM: "))
    destino = int(input("Digite o DESTINO: "))
    caminho = obtem_caminho(origem, destino, (grafo.BuscaLarga(origem))[1])
    custo = grafo.BuscaLarga(origem)[0][destino]
    resultados(origem, destino, caminho, custo)
elif aresta_negativa(grafo) == True:
    print("n")
    origem = int(input("Digite a ORIGEM: "))
    destino = int(input("Digite o DESTINO: "))
    caminho = obtem_caminho(origem, destino, (grafo.bellman_ford(origem))[1])
    custo = grafo.bellman_ford(origem)[0][destino]
    resultados(origem, destino, caminho, custo)
else:
    print("d")
    origem = int(input("Digite a ORIGEM: "))
    destino = int(input("Digite o DESTINO: "))
    caminho = obtem_caminho(origem, destino, (grafo.dijkstra(origem))[1])
    custo = grafo.dijkstra(origem)[0][destino]
    resultados(origem, destino, caminho, custo)




#start_time = time.time()
#print("Tempo de execucao: ", (time.time() - start_time))

#print("BELLMAN-FORD:", grafo.bellman_ford(0))
#print("DIJKSTRA:", grafo.dijkstra(0))
#print("LARGURA (DV):", grafo.busca_largura_alterado(0))
#print("LARGURA (GK):", grafo.BuscaLarga(0))