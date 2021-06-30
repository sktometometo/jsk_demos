import networkx as nx

class GraphEdge:

    def __init__(self,
                 node_id_from,
                 node_id_to,
                 behavior_type,
                 cost,
                 properties
                 ):
        self.node_id_from = node_id_from
        self.node_id_to = node_id_to
        self.behavior_type = behavior_type
        self.cost = cost
        self.properties = properties

class GraphNode:

    def __init__(self,
                 node_id,
                 properties
                 ):
        self.node_id = node_id
        self.properties = properties

class SupportBehaviorGraph:

    def __init__(self, edges={}, nodes={}):

        self._edges = {}
        self._nodes = {}
        self._network = nx.DiGraph()

        self.loadGraph( edges, nodes )

    def loadGraph(self, edges, nodes):
        """
        """

        for key, node in nodes.items():
            self._nodes[key] = node

        for edge in edges:
            self._edges[edge['from'],edge['to']] = edge
            self._network.add_edge(edge['from'],edge['to'],weight=edge['cost'])

    def calcPath(self, node_from, node_to):

        node_list = nx.shortest_path( self._network, node_from, node_to )
        path = []
        for index in range(len(node_list)-1):
            path.append(self._edges[node_list[index],node_list[index+1]])
        return path
