"""6.009 Lab 6 -- Gift Delivery."""


from graph import Graph

# NO ADDITIONAL IMPORTS ALLOWED!

permutations = []

def getPerms(l, path=[]):
    """ Get all permutations from a list of a list. Used as a helper 
    function for query. """
    global permutations
    if not l:
        permutations.append(path)
        return
    for item in l[0]:
        if item not in path:
            getPerms(l[1:], path+[item])

class GraphFactory:
    """Factory methods for creating instances of `Graph`."""

    def __init__(self, graph_class):
        """Return a new factory that creates instances of `graph_class`."""
        self.graphClass = graph_class

    def from_list(self, adj_list, labels=None):
        """Create and return a new graph instance.

        Use a simple adjacency list as source, where the `labels` dictionary
        maps each node name to its label.

        Parameters:
            `adj_list`: adjacency list representation of a graph
                        (as a list of lists)
            `labels`: dictionary mapping each node name to its label;
                      by default it's None, which means no label should be set
                      for any of the nodes

        Returns:
            new instance of class implementing `Graph`

        """
        graph = self.graphClass()
        #add the nodes
        for i in range(len(adj_list)):
            if labels == None:
                graph.add_node(i)
            else:
                graph.add_node(i, labels[i])
                
        #add the edges
        for i, node in enumerate(adj_list):
            for adj in node:
                graph.add_edge(i, adj)
        return graph

    def from_dict(self, adj_dict, labels=None):
        """Create and return a new graph instance.

        Use a simple adjacency dictionary as source where the `labels`
        dictionary maps each node name its label.

        Parameters:
            `adj_dict`: adjacency dictionary representation of a graph
            `labels`: dictionary mapping each node name to its label;
                      by default it's None, which means no label should be set
                      for any of the nodes

        Returns:
            new instance of class implementing `Graph`

        """
        graph = self.graphClass()
        #add the nodes
        for node in adj_dict:
            if labels is None:
                graph.add_node(node)
            else:
                graph.add_node(node, labels[node])
        
        #add the edges
        for node, l in adj_dict.items():
            for adj in l:
                graph.add_edge(node, adj)
        return graph

class SimpleGraph(Graph):
    """Simple implementation of the Graph interface."""

    def __init__(self):
        self.neighbor_dict = {} #node -> list of adjacent nodes neigh
        self.nodes_with_label = {} #label -> list of nodes 
        self.label_dict = {} #node -> label
    
    def query(self, pattern):
        #get a list of lists, where each sublist contains all the nodes 
        #matching the label given by pattern at the sublist's index
        nodesByLabel = []
        for tup in pattern:
            label = tup[0]
            if label == '*': 
                allNodes = list(self.neighbor_dict.keys())
                nodesByLabel.append(allNodes)
            elif label not in self.nodes_with_label:
                nodesByLabel.append([])
            else:
                nodesByLabel.append(self.nodes_with_label[label])
                
        #find potential solutions using a helper function                
        global permutations
        permutations = []
        getPerms(nodesByLabel)
                            
        #when a potential solution is made, check that the solution
        #is valid for the items in pattern
        result = []
        for p in permutations:
            valid = True
            for n in range(len(pattern)):
                for i in pattern[n][1]:
                    if p[i] not in self.neighbor_dict[p[n]]:
                        valid = False
            if valid:
                result.append(p)                
        return result
    
    def add_node(self, name, label=''):
        """Add a node with name `name` and label `label`."""
        if name in self.neighbor_dict:
            raise ValueError
        else:
            self.neighbor_dict[name] = []
            self.label_dict[name] = label
            if label not in self.nodes_with_label:
                self.nodes_with_label[label] = [name]
            else:
                self.nodes_with_label[label].append(name)

    def remove_node(self, name):
        """Remove the node with name `name`."""
        if name not in self.neighbor_dict:
            raise LookupError
        else:
            del self.neighbor_dict[name]
            label = self.label_dict[name]
            self.nodes_with_label[label].remove(name)
            del self.label_dict[name]

    def add_edge(self, start, end):
        """Add a edge from `start` to `end`."""
        if start not in self.neighbor_dict or end not in self.neighbor_dict:
            raise LookupError
        elif end in self.neighbor_dict[start]:
            raise ValueError
        else:
            self.neighbor_dict[start].append(end)

    def remove_edge(self, start, end):
        """Remove the edge from `start` to `end`."""
        if start not in self.neighbor_dict or end not in self.neighbor_dict or end not in self.neighbor_dict[start]:
            raise LookupError
        else:
            self.neighbor_dict[start].remove(end)

class CompactGraph(Graph):
    """Graph optimized for cases where many nodes have the same neighbors."""
    def __init__(self):
        self.label_dict = {} #node -> id
        self.setId = {frozenset([]):0} #frozenset -> id
        self.idSet = {0:frozenset([])} #id -> frozenset
        self.counter = 1 #ensures each id number is unique
        self.nodes_with_label = {} #label -> node

    def query(self, pattern):
        """Return a list of subgraphs matching `pattern`.

        Parameters:
            `pattern`: a list of tuples, where each tuple represents a node.
                The first element of the tuple is the label of the node, while
                the second element is a list of the neighbors of the node as
                indices into `pattern`. A single asterisk '*' in place of the
                label matches any label.

        Returns:
            a list of lists, where each sublist represents a match, its items
            being names corresponding to the nodes in `pattern`.

        """
        #get a list of lists, where each sublist contains all the nodes 
        #matching the label given by pattern at the sublist's index
        nodesByLabel = []
        for tup in pattern:
            label = tup[0]
            if label == '*': 
                allNodes = list(self.label_dict.keys())
                nodesByLabel.append(allNodes)
            elif label not in self.nodes_with_label:
                nodesByLabel.append([])

            else:
                nodesByLabel.append(self.nodes_with_label[label])
                
        #find potential solutions using a helper function                
        global permutations
        permutations = []
        getPerms(nodesByLabel)
                            
        #when a potential solution is made, check that the solution
        #is valid for the items in pattern
        result = []
        for p in permutations:
            valid = True
            for n in range(len(pattern)):
                for i in pattern[n][1]:
                    if p[i] not in self.idSet[self.label_dict[p[n]]]:
                        valid = False
            if valid:
                result.append(p)                
        return result

    def add_node(self, name, label=''):
        """Add a node with name `name` and label `label`."""
        if name in self.label_dict:
            raise ValueError
        else: 
            self.label_dict[name] = 0
            if label not in self.nodes_with_label:
                self.nodes_with_label[label] = set([name])
            else:
                self.nodes_with_label[label].add(name)

    def remove_node(self, name):
        """Remove the node with name `name`."""
        if name not in self.label_dict:
            raise LookupError
        else:
            for label, nodes in self.nodes_with_label.items():
                if name in nodes:
                    nodes.remove(name)
                if len(nodes) == 0:
                    del self.nodes_with_label[label]
            del self.label_dict[name]
            for adj, idnum in list(self.setId.items()).copy():
                if name in adj:
                    newset = set(adj)
                    newset.remove(name)
                    newset = frozenset(newset)
                    self.idSet[idnum] = newset
                    self.setId[newset] = idnum
                    del self.setId[adj]

    def add_edge(self, start, end):
        """Add a edge from `start` to `end`."""
        if start not in self.label_dict or end not in self.label_dict:
            raise LookupError
        elif end in self.idSet[self.label_dict[start]]:
            raise ValueError
        else:
            idnum = self.label_dict[start]
            newset = self.idSet[idnum]
            newset = set(newset)
            newset.add(end)
            newset = frozenset(newset)
            if newset not in self.setId:
                self.idSet[self.counter] = newset
                self.setId[newset] = self.counter
                self.label_dict[start] = self.counter
                self.counter += 1
            else:
                self.label_dict[start] = self.setId[newset]

    def remove_edge(self, start, end):
        """Remove the edge from `start` to `end`."""
        if start not in self.label_dict or end not in self.label_dict:
            raise LookupError
        elif end not in self.idSet[self.label_dict[start]]:
            raise LookupError
        else:
            idnum = self.label_dict[start]
            newset = self.idSet[idnum]
            newset = set(newset)
            newset.remove(end)
            newset = frozenset(newset)
            if newset not in self.setId:
                self.idSet[self.counter] = newset
                self.setId[newset] = self.counter
                self.label_dict[start] = self.counter
                self.counter += 1

def allocate_teams(graph, k, stations, gift_labels):
    """Compute the number of teams needed to deliver each gift.

    It is guaranteed that there is exactly one node for each gift type and all
    building nodes have the label "building".

    Parameters:
        `graph`: an instance of a `Graph` implementation
        `k`: minimum number of buildings that a cluster needs to contain for a
             delivery to be sent there
        `stations`: mapping between each node name and a string representing
                    the name of the closest subway/train station
        `gift_labels`: a list of gift labels

    Returns:
        a dictionary mapping each gift label to the number of teams
        that Santa needs to send for the corresponding gift to be delivered

    """
    #add undirected edge between all buildings that have the same station
    stationDict = {}
    for node, station in stations.items():
        if station not in stationDict:
            stationDict[station] = [node]
        else:
            stationDict[station].append(node)
    for l in stationDict.values():
        for i in l:
            for j in l:
                if i != j:
                    graph.add_edge(i,j)
    
    #use query to find maximal cliches, and uses them to build result
    result = {}
    for gift in gift_labels:
        i = 0
        while i == 0 or cliche:
            pattern = [(gift,[x for x in range(1,k+1)])]
            for n in range(1,k+i):
                attached = [x for x in range(k+i+1)]
                attached.remove(n)
                buildings = ('building',attached)
                pattern.append(buildings)
            cliche = graph.query(pattern)
            i += 1
        for tup in pattern:
            if i in tup[1]:
                tup[1].remove(i)
        cliche = graph.query(pattern[:-1])
        result[gift] = len(cliche)
    return result

if __name__ == '__main__':
    # Put code here that you want to execute when lab.py is run from the
    # command line, e.g. small test cases.
    
    #test SimpleGraph and from_dict
#    graph = {0:[1,3], 1:[2], 2:[0], 3:[]}
#    factory = GraphFactory(SimpleGraph)
#    inst = factory.from_dict(graph)
#    inst.add_node(4)
#    inst.add_edge(3,0)
#    inst.remove_node(4)
#    inst.remove_edge(3,0)
#    print('result:  ', inst.query([('*',[1,2]), ('*',[]), ('*',[])]))
#    print('expected: [[0, 1, 3], [0, 3, 1]]')
    
    pass
