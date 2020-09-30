import re
import networkx as nx
import matplotlib.pyplot as plt
import time


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






DATA_PATH = "dataset.dat"
DATA_PATH2 = "teste.dat"
data = open(DATA_PATH, "r")
grafo = Graph()

lines = data.readlines() # a list conteining all lines of our data set
regex = re.compile(r"[\w]+") # tratamento de string com regex

# preenchemos nosso grafo
for i in range(len(lines)):
    node = regex.findall(lines[i])
    grafo.add_node(node[0], node[1])
    grafo.add_node(node[1], node[0])




# Iniciando o Algoritmo baseado em BFS

start_bfs = time.time()
caminho_bfs = grafo.BFS("TelaLigouCercou","tulioalmeidars")





graph_bfs = nx.Graph()
for i in range (1,len(caminho_bfs)):
    graph_bfs.add_edge(caminho_bfs[i-1],caminho_bfs[i])

nx.draw(graph_bfs , with_labels = True)
plt.show()

end_bfs = time.time()
print( "Tempo de execução do Algoritmo BFS: ", end_bfs - start_bfs)








data.close()




