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
    

class Grafo:
    def __init__(self):
        self.dicionario = {}
    
    def add_edge(self, node, neighbor):
        if node not in self.dicionario:
            self.dicionario[node] = [neighbor]
        else:  # node is in our graph
            self.dicionario[node].append(neighbor)
    
    def BFS(self, start, goal):
        
        path = self.solve(start)
        
    
    def solve(self, start):
        queue = Queue()  # create our queue
        queue.enqueue(start) # add the starting point to the queue to find its neighbours
        prev = {}
        
        visited = {}  # a dict referecing our visited nodes
        for node in self.dicionario:
            visited[node] = False
        visited[start] = True
        
        while not queue.is_empty():
            node = queue.dequeue()
            neighbors = self.dicionario[node]
            
            for neighbor in neighbors:
                if not visited[neighbor]:
                    queue.enqueue(neighbor)
                    visited[neighbor] = True
        return path
    
    def reconstruct_path(self, start, goal, path):
        pass        
    
    def bfs2(self, start, goal):
        visited = {}
        level = {}
        parent = {}
        bfs_traversal_output = []
        queue = Queue()
        
        for node in self.dicionario:
            visited[node] = False
            parent[node] = None
            level[node] = -1
        
        #print(level)
        #print(parent)
        #print(visited)
        
        visited[start] = True
        level[start] = 0
        queue.enqueue(start)
        
        while not queue.is_empty():
            u = queue.dequeue()
            bfs_traversal_output.append(u)
            for v in self.dicionario[u]:
                if not visited[v]:
                    visited[v] = True
                    parent[v] = u
                    level[v] += level[u] + 1
        print(bfs_traversal_output)
        print("shortest distance: ", level[goal])
        
        path = []
        while v is not None:
            path.append(v)
            v = parent[v]
        path.reverse()
        print("caminho: ", path)
        
    
    def show_graph(self):
        for node in self.dicionario:
            print("{}:{}".format(node, dicionario[node]))


DATA_PATH2 = "teste.py"
data = open(DATA_PATH2, "r")
grafo = Grafo()

lines = data.readlines() # a list conteining all lines of our data set
regex = re.compile(r"[\w]+") # tratamento de string com regex

# preenchemos nosso grafo
for i in range(len(lines)):
    node = regex.findall(lines[i])
    grafo.add_edge(node[0], node[1])
    grafo.add_edge(node[1], node[0])

data.close()