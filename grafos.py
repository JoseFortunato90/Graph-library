import numpy as np
from collections import deque

class Grafo:
    
    def __init__(self, num_vertices, representacao='lista'):
        self.num_vertices = num_vertices
        self.representacao = representacao

        if representacao == 'lista':
            self.adj = [[] for _ in range(num_vertices)]
        elif representacao == 'matriz':
            self.adj = np.zeros((num_vertices, num_vertices), dtype=float)
        else:
             raise ValueError("Representação inválida. Use 'lista' ou 'matriz'.")
            

    def add_aresta(self,i,j,peso=1):
        
        if self.representacao == 'lista':
            self.adj[i].append((j, peso))
            self.adj[j].append((i, peso))

        if self.representacao == 'matriz':
            self.adj[i,j] = peso
            self.adj[j,i] = peso

    def get_arestas(self):

        arestas = []

        if self.representacao == 'lista':
            for u, vizinhos_de_u in enumerate(self.adj):
                for vizinho_info in vizinhos_de_u:
                    v = vizinho_info[0]
                    if u <= v:
                        arestas.append([u, v, vizinho_info[1]])

        elif self.representacao == 'matriz':
            for i in range(self.num_vertices):
                for j in range(i+1,self.num_vertices):
                    if self.adj[i,j] != 0:
                        arestas.append([i,j, self.adj[i,j]])


        return arestas
    
    def num_vertices(self):
        return self.num_aresta
    
    def num_arestas(self):
        
        num_arestas = 0
        if self.representacao == 'lista':

            for aresta in self.adj:
                num_arestas = num_arestas + len(aresta)

            num_arestas = num_arestas/2

        elif self.representacao == 'matriz':

            for i in range(self.num_vertices):
                for j in range(i+1,self.num_arestas):
                    if self.adj[i,j] != 0:
                        num_arestas = num_arestas + 1

        return num_arestas
    
    def viz(self, v):
        vizinhos = []

        if self.representacao == 'lista':
            for vizinho in self.adj[v]:
                vizinhos.append(vizinho[0])

        elif self.representacao == 'matriz':
            for j in range(self.num_vertices):
                if self.adj[v, j] != 0:
                    vizinhos.append(j)
        
        return vizinhos
    
    def grau(self,v):
        return len(self.viz(v))
    
    def peso(self, u, v):
        if self.representacao == 'lista':
            for aresta in self.adj[u]:
                if aresta[0] == v:
                    return aresta[1]
        
        elif self.representacao == 'matriz':
            return self.adj[u,v]
        
    def mind(self):
        menor = float('inf')
        for i in range(self.num_vertices):
            if menor > self.grau(i):
                menor = self.grau(i)

        return menor
    
    def maxd(self):
        maior = 0
        for i in range(self.num_vertices):
            if maior < self.grau(i):
                maior = self.grau(i)

        return maior
    
    def bfs(self,v):
        
        d = [float('inf')] * self.num_vertices
        d[v] = 0
        pi = [-1] * self.num_vertices
        
        Q = deque()
        Q.append(v)
        
        visitados = [False] * self.num_vertices
        visitados[v] = True

        
        while Q:
            u = Q.popleft()
                
            for aresta in self.viz(u):
                    
                if visitados[aresta] == False:
                    visitados[aresta] = True
                    Q.append(aresta)
                    d[aresta] = d[u] + 1
                    pi[aresta] = u
                
        return d, pi
    
    def dfs(self, v):
       
        pi = [-1] * self.num_vertices
        v_ini = [0] * self.num_vertices
        v_fim = [0] * self.num_vertices
        visitados = [False] * self.num_vertices
        
        tempo = 0 

        def dfs_recursivo(u):
            nonlocal tempo 
            
            visitados[u] = True
            tempo += 1
            v_ini[u] = tempo

            for vizinho_info in self.viz(u):
                vizinho = vizinho_info
                if not visitados[vizinho]:
                    pi[vizinho] = u
                    dfs_recursivo(vizinho)

            tempo += 1
            v_fim[u] = tempo

        dfs_recursivo(v)

        return pi, v_ini, v_fim
    

    def bf(self, v):
        d = [float('inf')] * self.num_vertices
        pi = [-1] * self.num_vertices
        d[v] = 0

        arestas = self.get_arestas()
        for i in range(self.num_vertices-1):
            for aresta in arestas:
                if d[aresta[1]] > d[aresta[0]] + aresta[2]:
                    d[aresta[1]] = d[aresta[0]] + aresta[2]
                    pi[aresta[1]] = aresta[0] 
         
       
        for aresta in arestas:
            if d[aresta[1]] > d[aresta[0]] + aresta[2]:
                return "Existem ciclos negativos"
                
        return d, pi



        
       
                    

g = Grafo(3, representacao='matriz')
g.add_aresta(0, 1, -4) 
g.add_aresta(0, 2, -2)
g.add_aresta(1, 2, -4) 
tempo = [(0, 0)] * g.num_vertices
visitados = [float('inf')] * g.num_vertices
print(g.bf(0))


