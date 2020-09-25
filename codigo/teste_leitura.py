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

    def bfs(self, start):
        queue = Queue()
        queue.enqueue(start)
        visited = {}
        previous = {}
        level = {}
        path = []
        for node in self.graph_dict:
            visited[node] = False
            previous[node] = None
            level[node] = -1
        visited[start] = True
        level[start] = 0
        # print(visited)
        # print(previous)
        # print(level)

        while not queue.is_empty():
            node = queue.dequeue()
            path.append(node)
            # get the neoghbours of the current node
            neighbours = self.graph_dict[node]
            for neighbour in neighbours:
                if not visited[neighbour]:
                    queue.enqueue(neighbour)
                    visited[neighbour] = True
                    previous[neighbour] = node
                    level[neighbour] = level[node] + 1
        return previous
        

# será melhor gerar as funções de busca de menor caminho fora da classe graph
# Decidir as funções, por enquanto BFS;

DATA_PATH = "dataset.dat"
data = open(DATA_PATH, "r")
grafo = Graph()

lines = data.readlines() # a list conteining all lines of our data set
regex = re.compile(r"[\w]+") # tratamento de string com regex

# preenchemos nosso grafo

for i in range(len(lines)):
    node = regex.findall(lines[i])
    grafo.add_node(node[0], node[1])
    grafo.add_node(node[1], node[0])

data.close()