import re

#Teste

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
    
    def Dijkstra(self, start, goal):
        INFINITY = 9999999999
        shortest_distance = {}
        track_predecessor = {}
        unseen_nodes = graph
        path = []
        
        for node in unseen_nodes:
            shortest_distance[node] = INFINITY
        shortest_distance[start] = 0

        while unseen_nodes:
            min_distance_node = None
            for node in unseen_nodes:
                if min_distance_node is None:
                    min_distance_node = node
                elif shortest_distance[node] < shortest_distance[min_distance_node]:
                    min_distance_node = node
            print(min_distance_node)

            path_option = graph[min_distance_node].items()

            for child_node, weight in path_option:
                if weight + shortest_distance[min_distance_node] < shortest_distance[child_node]:
                    shortest_distance[child_node] = weight + \
                        shortest_distance[min_distance_node]
                    track_predecessor[child_node] = min_distance_node


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