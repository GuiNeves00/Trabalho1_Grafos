class Grafo:

  def __init__(self, num_vert = 0, num_arestas = 0, lista_adj = None, mat_adj = None):
    self.num_vert = num_vert
    self.num_arestas = num_arestas
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
    dist = [float("inf") for v in range (self.num_vert)] #L1~2: inicializa o vet. dist com valor infinito em cada pos.
    pred = [None for v in range (self.num_vert)] #L3: inicializa o vet. pred com None em cada pos.
    
    dist [s] = 0 #L4

    Q = [v for v in range(self.num_vert)] #L5: inicializa Q com os vertices do grafo. Q smp possui os vertices nao computados
    u = 999 #incializacao do u
    while Q != []: #L6
      for i in range(len(dist)): #L7: i percorre dist 
        if Q[i] != None and dist[i] < u: #L7: checa se o vértice i já foi computado e verifica o menor valor de dist
          u = dist[i] #L7: u recebe o menor valor de dist
      #Q.pop(u) #L8: remove u de Q

      #fazer depois: "se dado grafo tiver mtts vertices, usar mat. caso contrario, usar lista"

      for v in dist[u][v]:
        print(self.mat_adj[u][v])
      break

      
    print("Q=", Q)
    print("u=", u) 