from pickle import TRUE
import sys
sys.path.append("..")
import tp1_grafo as GrafoT1
import time

def eh_ponderado(Grafo):
    """Retorna True se o grafo for ponderado, ou False caso contrario"""
    ponderado = False
    for i in range(len(Grafo.lista_adj)):
        for j in range(len(Grafo.lista_adj[i])):
            if Grafo.lista_adj[i][j][1] != 1:
                return True
            elif Grafo.lista_adj[i][j][1] == 1:
                ponderado = False
    return ponderado

def obter_caminho(s, t, pred):
    """Obtem o caminho minimo a partir do vetor pred, que eh retornado nos algoritmos de Busca em Largura, Bellman-Ford e Dijkstra"""
    caminho = [t]
    x = t
    #Itera ate encontrar a origem, assim obtendo todo o caminho
    while x != s:
        x = pred[x]
        caminho.append(x)
    #Inverte o caminho encontrado anteriormente, pois a ordem do vetor pred eh inversa
    caminho.reverse()
    return caminho

def resultados(origem, destino, caminho, custo):

    print("Origem: ", origem)
    print("Destino: ", destino)
    print("Caminho: ", caminho)
    print("Custo: ", custo)

grafo = GrafoT1.Grafo()
nome_arq = input("Digite o nome do arquivo: ")
print("Lendo o Arquivo...")
grafo.ler_arquivo(nome_arq)
print("Escolhendo Algoritmo...")

if eh_ponderado(grafo) == False:
    print("\033[1;33;40mBUSCA EM LARGURA")
    origem = int(input("Digite a ORIGEM: "))
    destino = int(input("Digite o DESTINO: "))
    print("--------------------\nProcessando...")
    caminho = obter_caminho(origem, destino, (grafo.busca_largura_t1(origem))[1]) #Obtem caminho a partir de origem, destino e pred. busca_largura_t1[1] = vetor pred
    start_time = time.time() #Inicia o timer para contar o tempo de execucao do algoritmo
    custo = grafo.busca_largura_t1(origem)[0][destino] #busca_largura_t1 retorna uma tupla, onde [0] = vetor dist. Enquanto [0][destino] obtem o custo do caminho minimo para destino
    resultados(origem, destino, caminho, custo) #Imprime resultados
    print("Tempo: %.10f"  % (time.time() - start_time)) #Imprime o tempo de execucao do algoritmo
elif grafo.aresta_negativa == True:
    print("\033[1;35;40mBELLMAN-FORD")
    origem = int(input("Digite a ORIGEM: "))
    destino = int(input("Digite o DESTINO: "))
    print("--------------------\nProcessando...")
    caminho = obter_caminho(origem, destino, (grafo.bellman_ford(origem))[1]) #Obtem caminho a partir de origem, destino e pred. bellman_ford[1] = vetor pred
    start_time = time.time() #Inicia o timer para contar o tempo de execucao do algoritmo
    custo = grafo.bellman_ford(origem)[0][destino] #bellman_ford retorna uma tupla, onde [0] = vetor dist. Enquanto [0][destino] obtem o custo do caminho minimo para destino
    resultados(origem, destino, caminho, custo) #Imprime resultados
    print("Tempo: %.10f"  % (time.time() - start_time)) #Imprime o tempo de execucao do algoritmo
else:
    print("\033[1;36;40mDIJKSTRA")
    origem = int(input("Digite a ORIGEM: "))
    destino = int(input("Digite o DESTINO: "))
    print("--------------------\nProcessando...")
    caminho = obter_caminho(origem, destino, (grafo.dijkstra(origem))[1]) #Obtem caminho a partir de origem, destino e pred. dijkstra[1] = vetor pred
    start_time = time.time() #Inicia o timer para contar o tempo de execucao do algoritmo
    custo = grafo.dijkstra(origem)[0][destino] #dijkstra retorna uma tupla, onde [0] = vetor dist. Enquanto [0][destino] obtem o custo do caminho minimo para destino
    resultados(origem, destino, caminho, custo) #Imprime resultados
    print("Tempo: %.10f"  % (time.time() - start_time)) #Imprime o tempo de execucao do algoritmo
print("--------------------")



#FUNCOES ANTIGAS
#def aresta_negativa(Grafo):
#    """Retorna True se o grafo possuir aresta negativa, ou False caso contrario"""
#    for i in range(len(Grafo.lista_adj)):
#        for j in range(len(Grafo.lista_adj[i])):
#            if Grafo.lista_adj[i][j][1] < 0: #Ao encontrar uma aresta negativa, encerra a funcao retornando True
#                return True
#    return False #Se nenhum foi encontrada, retorna True