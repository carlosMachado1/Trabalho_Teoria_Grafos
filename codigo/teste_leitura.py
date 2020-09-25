import re

/Teste

class Graph:
    def __init__(self):
        # implementar dessa maneira ou com hash?
        
        self.graph = {}
    
    def add_node(self, node, neighbor):
        if node not in self.graph:
            self.graph[node] = [neighbor]
        else:  # node in graph
            self.graph[node].append(neighbor)
    
    def show_graph(self):
        i = 0
        for node in self.graph:
            if i == 200:
                break
            print(node)
            i += 1
            

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