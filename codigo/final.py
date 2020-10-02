import re
import networkx as nx
import matplotlib.pyplot as plt
import time


#INICIO GRAFO PARA O A*---------------------------------------------------------------------------------
class Graphy:
    def __init__(self, graph_dict=None, directed=True):
        self.graph_dict = graph_dict or {}
        self.directed = directed
        if not directed:
            self.make_undirected()

            
    #Deixando o grafo não direcionado----
    def make_undirected(self):
        for a in list(self.graph_dict.keys()):
            for (b, dist) in self.graph_dict[a].items():
                self.graph_dict.setdefault(b, {})[a] = dist

    # Adicionar um link de A para B de distancia q, e de B para A
    def connect(self, A, B, distance=1):
        self.graph_dict.setdefault(A, {})[B] = distance
        if not self.directed:
            self.graph_dict.setdefault(B, {})[A] = distance


    # Obtendo os vizinhos
    def get(self, a, b=None):
        links = self.graph_dict.setdefault(a, {})
        if b is None:
            return links
        else:
            return links.get(b)

class Node:

    def __init__(self, name:str, parent:str):
        self.name = name
        self.parent = parent
        self.g = 0 # distancia para o nó inicial
        self.h = 0 # Distancia para o nó final
        self.f = 0 # custo total


    
    def __eq__(self, other):
        return self.name == other.name
    # Sort nodes
    def __lt__(self, other):
         return self.f < other.f
    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.name, self.f)) 
# A* search
def astar_search(graph, heuristics, start, end):
        # Cria o Node do nó inicial e do nó final, e as listas aberta e fechada
    open = []
    closed = []

    start_node = Node(start, None)
    goal_node = Node(end, None)
    
    open.append(start_node)    # Adiciona o Nó inicial na lista aberta
    
    # Enquanto a lista aberta não esta vazia
    while len(open) > 0:
        # Ordena a lista aberta para usarmos o nó com menor custo
        open.sort()

        # Usamos o nó com menor custo e adicionamos ele na lista fechada
        current_node = open.pop(0)
        closed.append(current_node)
        
        # Verifica se chegamos no nó final
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.name + ': ' + str(current_node.g))
                current_node = current_node.parent
            path.append(start_node.name + ': ' + str(start_node.g))

            # Retorna o caminho inverso
            return path[::-1]
        # Vizinhos
        neighbors = graph.get(current_node.name)
        # percorre os vizinhos
        for key, value in neighbors.items():
            
            neighbor = Node(key, current_node)     # Cria um Node do vizinho
            
            
            if(neighbor in closed):       # Verifica se o vizinho já está na lista fechada
                continue
            # Verifica os custos
            neighbor.g = current_node.g + graph.get(current_node.name, neighbor.name)
            neighbor.h = heuristics.get(neighbor.name)
            neighbor.f = neighbor.g + neighbor.h

            
            # Verifica se o vizinho esta na lista aberta e se tem um custo total menor
            if(add_to_open(open, neighbor) == True):
                
                open.append(neighbor)    # Se positivo, adiciona o vizinho na lista aberta

                
    # Se não houver caminho disponivel, retorna None
    return None
# Verifica se o vizinho pode ser adicionado na lista aberta, para não ter itens repetidos
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f > node.f):
            return False
    return True


#FIM DAS CLASSES PARA A*----------------------------------------------------------------------------------








class Queue:
    def __init__(self):
        self.queue = []

    def enqueue(self, item):
        self.queue.append(item)

    def dequeue(self):
        if len(self.queue) < 1:
            return None
        return self.queue.pop(0)

    def display(self):
        print(self.queue)

    def size(self):
        return len(self.queue)

    def front(self):
        if len(self.queue) < 1:
            return None
        return self.queue[0]

    def is_empty(self):
        return len(self.queue) == 0





class Graph:
    def __init__(self):
        self.graph_dict = {}

    def add_node(self, node, neighbour):
        if node not in self.graph_dict:
            self.graph_dict[node] = [neighbour]
        else:
            self.graph_dict[node].append(neighbour)

    def show_graph(self):
        print(self.graph_dict)
        for node in self.graph_dict:
            print('{}:{}'.format(node, self.graph_dict[node]))

    def BFS(self, start, goal):
        queue = Queue() # criar a fila
        queue.enqueue(start) # começando a busca pelo elemento inicial
        
        visited = {}
        previous = {} #
        level = {}
        bfs_traversal = []
        
        for node in self.graph_dict:
            visited[node] = False
            previous[node] = None
            level[node] = -1
            
        visited[start] = True
        level[start] = 0

        while not queue.is_empty():
            node = queue.dequeue() # dequeuing the node
            bfs_traversal.append(node)
            # get the neoghbours of the current node
            neighbors = self.graph_dict[node]
            for neighbor in neighbors:
                if not visited[neighbor]:
                    queue.enqueue(neighbor)
                    visited[neighbor] = True
                    previous[neighbor] = node
                    level[neighbor] = level[node] + 1
        
        #print("caminho feito: ", bfs_traversal)
        print("Arestas percorridas até o ponto de parada: ", level[goal])
        #print("nó parentses: ", previous)
        
        path = []
        start_bak = goal
        while start_bak is not None:
            path.append(start_bak)
            start_bak = previous[start_bak]
        path.reverse()
        print("caminho: ", path)
        return path

def Dijkstra(graph, start, goal):
    INFINITY = 9999999999
    shortest_distance = {}
    track_predecessor = {}
    unseen_nodes = graph
    path = []

    for node in unseen_nodes:
        shortest_distance[node] = INFINITY
    shortest_distance[start] = 0
        #print(shortest_distance)
        
    while unseen_nodes:
        min_distance_node = None
        for node in unseen_nodes:
            if min_distance_node is None:
                    
                min_distance_node = node
                #print (shortest_distance[min_distance_node])
            elif shortest_distance[node] < shortest_distance[min_distance_node]:
                min_distance_node = node
            #print(min_distance_node)

        path_option = graph[min_distance_node].items()
            #print(path_option)

        for child_node, weight in path_option:
            if weight + shortest_distance[min_distance_node] < shortest_distance[child_node]:
                shortest_distance[child_node] = weight + \
                    shortest_distance[min_distance_node]
                track_predecessor[child_node] = min_distance_node

        unseen_nodes.pop(min_distance_node)

    current_node = goal

    while current_node:
        try:
            path.insert(0, current_node)
            current_node = track_predecessor[current_node]
        except KeyError:
                # print(path)
            break

    if shortest_distance[goal] != INFINITY:
        print("Arestas percorridas até o ponto de parada: " + str(shortest_distance[goal]))
        print("Optimal path is" + str(path))
    return (path)






DATA_PATH = "dataset.dat"
DATA_PATH2 = "teste.dat"
data = open(DATA_PATH, "r")
grafo = Graph()
grafo_dois = Graphy()

lines = data.readlines() # a list conteining all lines of our data set
regex = re.compile(r"[\w]+") # tratamento de string com regex
heuristics = {}
# preenchemos nosso grafo
for i in range(len(lines)):
    node = regex.findall(lines[i])
    grafo.add_node(node[0], node[1])
    grafo.add_node(node[1], node[0])
    grafo_dois.connect(node[0],node[1],1)    
    

    heuristics[node[0]] = 1
    heuristics[node[1]] = 1

data.close()





    




print( "--------Executando BFS--------")
start_bfs = time.time()
caminho_bfs = grafo.BFS("TelaLigouCercou","tulioalmeidars")
end_bfs = time.time()
print( "Tempo de execução do Algoritmo BFS: ", end_bfs - start_bfs)




print("\n\n--------Executando Dijkstra--------")


base={}
novo = {}
for node in grafo.graph_dict:
    
    for arestas in grafo.graph_dict[node]:
        base[arestas] = 1
    novo[node] = base
    base = {}

start_dijk = time.time()
#Dijkstra(novo,"TelaLigouCercou","tulioalmeidars")
end_dijk = time.time()

print( "Tempo de execução do Algoritmo Dijkstra: ",end_dijk - start_dijk)


    



print( "\n\n---------Executando A*-----------")
grafo_dois.make_undirected()


start_astar = time.time()
pathy = astar_search(grafo_dois, heuristics, "TelaLigouCercou", "tulioalmeidars")
print(pathy)

end_astar = time.time()

print( " Tempo de execução do Algoritmo A*: ", end_astar - start_astar)


graph_bfs = nx.Graph()
for i in range (1,len(caminho_bfs)):
    graph_bfs.add_edge(caminho_bfs[i-1],caminho_bfs[i])

nx.draw(graph_bfs , with_labels = True)
plt.show()

