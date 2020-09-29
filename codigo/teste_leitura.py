import re

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
        
        print("caminho feito: ", bfs_traversal)
        print("menor caminho até o ponto de parada: ", level[goal])
        print("nó parentses: ", previous)
        
        path = []
        while node is not None:
            path.append(node)
            node = previous[node]
        path.reverse()
        print("caminho: ", path)
    

DATA_PATH = "dataset.dat"
DATA_PATH2 = "teste"
data = open(DATA_PATH2, "r")
grafo = Graph()

lines = data.readlines() # a list conteining all lines of our data set
regex = re.compile(r"[\w]+") # tratamento de string com regex

# preenchemos nosso grafo
for i in range(len(lines)):
    node = regex.findall(lines[i])
    grafo.add_node(node[0], node[1])
    grafo.add_node(node[1], node[0])

data.close()