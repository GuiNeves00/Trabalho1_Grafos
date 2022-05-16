import sys
sys.path.append("..")
import tp1_grafo as grafoT1

grafo = grafoT1.Grafo()
grafo.ler_arquivo("toy.txt")
print(grafo.dijkstra(0))