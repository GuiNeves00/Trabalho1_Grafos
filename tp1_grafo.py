from math import dist
from re import X
from xml.etree.ElementPath import prepare_descendant

class Grafo:
  #CHANGED: aresta_negativa -> verificar se existe aresta negativa no grafo
  def __init__(self, num_vert = 0, num_arestas = 0, aresta_negativa = None, lista_adj = None, mat_adj = None):
    self.num_vert = num_vert
    self.num_arestas = num_arestas
    self.aresta_negativa = aresta_negativa #CHANGED
    if lista_adj is None:
      self.lista_adj = [[] for i in range(num_vert)]
    else:
      self.lista_adj = lista_adj
    if mat_adj is None:
      self.mat_adj = [[0 for j in range(num_vert)] for i in range(num_vert)]
    else:
      self.mat_adj = mat_adj
      

  def add_aresta(self, u, v, w = 1):
    """Adiciona aresta de u a v com peso w"""
    self.num_arestas += 1
    if u < self.num_vert and v < self.num_vert:
      self.lista_adj[u].append((v, w))
      self.mat_adj[u][v] = w
    else:
      print("Aresta invalida!")

  def remove_aresta(self, u, v):
    """Remove aresta de u a v, se houver"""
    if u < self.num_vert and v < self.num_vert:
      if self.mat_adj[u][v] != 0:
        self.num_arestas -= 1
        self.mat_adj[u][v] = 0
        for (v2, w2) in self.lista_adj[u]:
          if v2 == v:
            self.lista_adj[u].remove((v2, w2))
            break
      else:
        print("Aresta inexistente!")
    else:
      print("Aresta invalida!")

  def grau(self, u):
    """Retorna o grau do vertice u"""
    return len(self.lista_adj[u])

  def adjacente(self, u, v):
    """Determina se v é adjacente a u"""
    if self.mat_adj[u][v] != 0:
      return True
    else:
      return False

  def adjacentes_peso(self, u):
    """Retorna a lista dos vertices adjacentes a u no formato (v, w)"""
    return self.lista_adj[u]

  def adjacentes(self, u):
    """Retorna a lista dos vertices adjacentes a u"""
    adj = []
    for i in range(len(self.lista_adj[u])):
      (v, w) = self.lista_adj[u][i]
      adj.append(v)
    return adj

  def densidade(self):
    """Retorna a densidade do grafo"""
    return self.num_arestas / (self.num_vert * (self.num_vert - 1))

  def subgrafo(self, g2):
    """Determina se g2 e subgrafo de self"""
    if g2.num_vert > self.num_vert:
      return False
    for i in range(len(g2.mat_adj)):
      for j in range(len(g2.mat_adj[i])):
        if g2.mat_adj[i][j] != 0 and self.mat_adj[i][j] == 0:
          return False
    return True

  def ler_arquivo(self, nome_arq):
    """Le arquivo de grafo no formato dimacs"""
    try:
      arq = open(nome_arq)
      #Leitura do cabecalho
      str = arq.readline()
      str = str.split(" ")
      self.num_vert = int(str[0])
      cont_arestas = int(str[1])
      #Inicializacao das estruturas de dados
      self.lista_adj = [[] for i in range(self.num_vert)]
      self.mat_adj = [[0 for j in range(self.num_vert)] for i in range(self.num_vert)] 
      #Le cada aresta do arquivo
      for i in range(0,cont_arestas):
        str = arq.readline()
        str = str.split(" ")
        u = int(str[0]) #Vertice origem
        v = int(str[1]) #Vertice destino
        w = int(str[2]) #Peso da aresta
        self.add_aresta(u, v, w)
        #CHANGED: identifica se existe aresta negativa no grafo
        if w < 0:
          self.aresta_negativa = True

    except IOError:
      print("Nao foi possivel encontrar ou ler o arquivo!")


  def busca_largura(self, s):
    """Retorna a ordem de descoberta dos vertices pela 
      busca em largura a partir de s"""
    desc = [0 for v in range(self.num_vert)]
    Q = [s]
    R = [s]
    desc[s] = 1
    while Q:
      u = Q.pop(0)
      for (v, w) in self.lista_adj[u]:
        if desc[v] == 0:
          Q.append(v)
          R.append(v)
          desc[v] = 1
    return R
  def busca_profundidade(self, s):
    """Retorna a ordem de descoberta dos vertices pela 
      busca em profundidade a partir de s"""
    desc = [0 for v in range(self.num_vert)]
    S = [s]
    R = [s]
    desc[s] = 1
    while S:
      u = S[-1]
      desempilhar = True
      for (v, w) in self.lista_adj[u]:
        if desc[v] == 0:
          desempilhar = False
          S.append(v)
          R.append(v)
          desc[v] = 1
          break
      if desempilhar:
        S.pop(-1)
    return R


  
  def busca_profundidade_rec(self, s, R, desc):
    """Retorna a ordem de descoberta dos vertices pela 
      busca em profundidade a partir de s"""
    R.append(s)
    desc[s] = 1
    for (v, w) in self.lista_adj[s]:
      if desc[v] == 0:
        self.busca_profundidade_rec(v, R, desc)
    return R
    
  def conexo(self, s):
    """Retorna Ture se o grafo e conexo e False caso contrario
      baseado na busca em largura"""
    desc = [0 for v in range(self.num_vert)]
    Q = [s]
    R = [s]
    desc[s] = 1
    while Q:
      u = Q.pop(0)
      for (v, w) in self.lista_adj[u]:
        if desc[v] == 0:
          Q.append(v)
          R.append(v)
          desc[v] = 1
    for i in range(len(desc)):
      if desc[i] == 0:
        return False
    return True

  def ciclo(self, s):
    """Retorna Ture se o grafo tem ciclo e False caso contrario
      baseado na busca em largura"""
    desc = [0 for v in range(self.num_vert)]
    for s in range(self.num_vert):
      if desc[s] == 0:
        Q = [s]
        R = [s]
        desc[s] = 1
        while Q:
          u = Q.pop(0)
          for (v, w) in self.lista_adj[u]:
            if desc[v] == 0:
              Q.append(v)
              R.append(v)
              desc[v] = 1
            else:
              return True
    return False

  def dijkstra(self, s):
    """Obtem caminho minimo de s para todos os vertices do grafo (apenas em grafos SEM arestas negativas)."""
    dist = [float('inf') for v in range (self.num_vert)] #L1~2: inicializa o vet. dist com valor infinito em cada pos.
    pred = [None for v in range (self.num_vert)] #L3: inicializa o vet. pred com None em cada pos.
    dist[s] = 0 #Distancia para origem eh 0
    Q = [False for v in range(self.num_vert)] #L5: Inicializa Q = False. os indices de Q representam os vert., os que foram explroados = True, nao explorados = False
    u = s

    while True: #L6
      atualizou = False
      menor_dist = float('inf') #menor_dist inicia com infinito porque as distancias ainda sao desconhecidas, exceto a origem. Assim tornando possível a busca pelo menor valor do vetor dist, 
                                #pois, se o valor de menor_dist for >= que o do indice atual, logo o indice atual sera menor e então sera atribuido o valor, encontrando o "novo menor".
      for i in range(len(Q)): #percorre o vetor Q para achar um novo u
        if menor_dist >= dist[i] and Q[i] != True:
          menor_dist = dist[i]
          u = i #u recebe o indice que representa o vert. de menor distancia no vetor dist.
          atualizou = True
        elif i == len(Q)-1 and atualizou == False: #se o vetor Q ja foi inteiramente percorrido e u nao atualizou, significa q ja podemos encerrar o algoritmo
          return dist, pred
          
      Q[u] = True #Equivalente a Q.pop(u)
      
      #Verifica-se qual a melhor opcao para usar entre matriz ou lista
      if (self.num_vert < 1000):
      #MATRIZ
        for v in range(self.num_vert): #dentro desse laço, percorremos o grafo em formato de matriz de adj (num_vert = total de repeticoes necessarias, pq percorremos toda a matriz).
            #u representa a linha da matriz que eh o vert. de menor distancia do vetor dist. isso faz com que percorremos apenas o "necessario".
          if self.mat_adj[u][v] != 0 and dist[v] > dist[u] + self.mat_adj[u][v]:
            dist[v] = dist[u] + self.mat_adj[u][v]
            pred[v] = u
      else:
      #LISTA
        for (v, w) in self.lista_adj[u]:
          if dist[v] > dist[u] + w:
            dist[v] = dist[u] + w
            pred[v] = u

  def bellman_ford (self, s):
    """Obtem o caminho minimo de s para todos os vertices do grafo (funciona para grafos com arestas negativas)."""
    dist = [float('inf') for v in range (self.num_vert)] #Inicializa vetor dist com infinito em cada pos.
    pred = [None for v in range (self.num_vert)] #Inicializa vetor pred com None em cada pos.
    dist[s] = 0 #Distancia para origem eh 0
    E = [(None, None, None) for x in range(self.num_arestas)] #Inicializa uma lista de tuplas que contera as arestas e seus respectivos pesos. indice[0][1] possuem as arestas e o [2] o peso
    x = 0 #Variavel auxiliar para ajudar a atribuir valores na lista E

    #Obtem os valores da lista E (arestas e peso)
    for i in range(len(self.mat_adj)):
      for j in range(len(self.mat_adj[i])):
        if self.mat_adj[i][j] != 0: #Se existir adjacencia...
          E[x] = (i, j, self.mat_adj[i][j]) #Lista E recebe aresta ij e peso
          x = x + 1

    #Laco principal
    for i in range(self.num_vert - 1): #Percorremos todas os vertices do grafo
      trocou = False #Flag para encerrar o algoritmo mais cedo, salvando custos
      for (u, v, w) in E: #Percorremos todas as arestas dos vertices do grafo
        if dist[v] > dist[u] + w: #Se encontrar um caminho melhor atraves de determinada aresta...
          dist[v] = dist[u] + w
          pred[v] = u
          trocou = True #Atualiza flag, sinalizando que houve troca

      if trocou == False: #Se percorremos todo os vertices/arestas e nao houve troca, significa que podemos encerrar o algoritmo
        return dist, pred

  def busca_largura_t1(self, s):
    """Descobre o caminho minimo de s para todos os vertices em grafos nao ponderados/Descobre se existe caminho de s a t"""
    dist = [float('inf') for v in range(self.num_vert)] #Inicializa a função com todas as distancias do vertice vert recebendo infinito
    pred = [None for v in range(self.num_vert)] #Predecessores no vertice vert recebem null pois não se sabe o caminho minimo
    dist[s] = 0   #A distancia da origem para ela mesma vai ser 0
    Q = [s] #A lista q recebe o vetor de origem s

    while len(Q) != 0: #Enquanto o tamanho da lista q for diferente de 0
      u = Q.pop(0) #Remove o primeiro elemento de q e adciona ele em u
      for (v, w) in self.lista_adj[u]: #para cada vert na lista de adj de u faz:
        if dist[v] == float('inf'): #se a distancia do vert = inf significa q não foi percorrido o no,dessa forma ira percorrer
          Q.append(v) #Adciona o vert no final da lista q
          dist[v] = dist[u] + 1 #Se atualiza a distancia do vert passando por u+1
          pred[v] = u #O predecessor do vert é atualizado recebendo o valor de u
    return dist, pred #Retorna a distancia e os predecessores