"""6.009 Lab 6 -- Gift Delivery."""


from graph import Graph

# NO ADDITIONAL IMPORTS ALLOWED!

permutations = []



class GraphFactory:
    """Factory methods for creating instances of `Graph`."""

    def __init__(self, graph_class):
        """Return a new factory that creates instances of `graph_class`."""
        self.graph = graph_class

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
        new_graph = self.graph()
        for node in range(len(adj_list)): #add all nodes
            if labels != None:
                new_graph.add_node(node, labels[node])
            else:
                new_graph.add_node(node)
        for i, node in enumerate(adj_list): #add all edges
            for adj in node:
                new_graph.add_edge(i, adj)
        return new_graph

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
        new_graph = self.graph()
        for node in adj_dict: #add all nodes
            
            if labels != None:
                new_graph.add_node(node, labels[node])
            else:
                new_graph.add_node(node)
                
        for node in adj_dict: #add all neighbors
            neighbors = adj_dict[node]
            
            for edge_end in neighbors:
                new_graph.add_edge(node, edge_end)
        return new_graph


class SimpleGraph(Graph):
    """Simple implementation of the Graph interface."""
    def __init__(self):
        self.label_dict = dict() # will be of form {node:label,}
        self.neighbor_dict = dict() #will be of form {node: [neighbor, neighbor]}
        self.nodes_with_label = dict() #will be of form {label:[node, node, ...]}
        
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
        output = []

        edge_reqs = []
        label_list = []
        for pattern_elem in pattern: #create list of all possible combinations of buildings with the right labels
            label_list.append(pattern_elem[0])
        all_possibilities = label_possibilities(self.label_dict, label_list)

        for possibility in all_possibilities: # if the possib fits the model
            if test_possib(possibility, pattern, self.neighbor_dict) == True:
                if len(set(possibility)) == len(possibility):
                    output.append(possibility)
            
        return output

    def add_node(self, name, label=''):
        """Add a node with name `name` and label `label`."""
        if name not in self.label_dict:
            self.label_dict[name] = label
            self.neighbor_dict[name] = []
            if label in self.nodes_with_label:
                self.nodes_with_label[label].append(name)
            else:
                self.nodes_with_label[label] = [name]
        else:
            raise ValueError

    def remove_node(self, name):
        """Remove the node with name `name`."""
        if name in self.label_dict:
            name_label = self.label_dict[name]
            del self.label_dict[name]
            del self.neighbor_dict[name]
            self.nodes_with_label[name_label].remove(name)
            
        else:
            raise LookupError

    def add_edge(self, start, end):
        """Add a edge from `start` to `end`."""
        #print(self.neighbor_dict)
        if start not in self.neighbor_dict:
            raise LookupError
        elif end not in self.neighbor_dict:
            raise LookupError
        elif end not in self.neighbor_dict[start]:
            self.neighbor_dict[start].append(end)
        else:
            raise ValueError

    def remove_edge(self, start, end):
        """Remove the edge from `start` to `end`."""
        if start in self.neighbor_dict:
            if end in self.neighbor_dict[start]:
                self.neighbor_dict[start].remove(end)
            else:
                raise ValueError
        else:
            raise LookupError


class CompactGraph(Graph):
    """Graph optimized for cases where many nodes have the same neighbors."""
    

    def __init__(self):
        self.neighbors_to_nodes = {frozenset(): set()}
        self.label_dict = {}
        self.list_of_nodes = list()
        
        pass

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
        label_list = []
        for pattern_elem in pattern:
            label_list.append(pattern_elem[0])
        all_possibilities = label_possibilities(self.label_dict, label_list)
        filter_possib = []
        for i in all_possibilities:
            if len(set(i)) == len(i):
                filter_possib.append(i)  
        output = []
        sorted_ntn = sorted(self.neighbors_to_nodes.items(), key = lambda x : len(x[1]), reverse = True)
        for trial in filter_possib:
            if test_possib_c(trial, pattern, sorted_ntn):
                output.append(trial)
        return output
     


    def add_node(self, name, label=''):
        """Add a node with name `name` and label `label`."""
        if name not in self.label_dict:
            self.label_dict[name] = label
            self.neighbors_to_nodes[frozenset([])] = set([name])
            self.list_of_nodes.append(name)
        else:
            raise ValueError


    def remove_node(self, name):
        """Remove the node with name `name`."""
        if name in self.label_dict:
            del self.label_dict[name]
            self.list_of_nodes.remove(name)
            new_ntn = self.neighbors_to_nodes.copy()
            for data in self.neighbors_to_nodes:
                if data == frozenset([name]):
                    del new_ntn[frozenset([name])]
                elif name in data:
                    val = new_ntn[data]
                    setdata = set(data)
                    setdata.remove(name)
                    setdata = frozenset(setdata)
                    del new_ntn[data]
                    new_ntn[setdata] = val
            self.neighbors_to_nodes = new_ntn

            
        else:
            raise LookupError

    def add_edge(self, start, end):
        """Add a edge from `start` to `end`."""
        #print(self.neighbors_to_nodes)
        if start not in self.label_dict:
            raise LookupError
        elif end not in self.label_dict:
            raise LookupError
        new_start = True
        new_neighbors = None
        for neighbors in self.neighbors_to_nodes:
            if start in self.neighbors_to_nodes[neighbors]:
                new_start = False
                new_neighbors = neighbors
                break
        if new_start == False:
            setn = set(new_neighbors)
            setn.add(end)
            potenfsetneigh = frozenset(setn)
            try:
                self.neighbors_to_nodes[potenfsetneigh].add(start)
                self.neighbors_to_nodes[new_neighbors].remove(start)
                if self.neighbors_to_nodes[new_neighbors] == set():
                    del self.neighbors_to_nodes[new_neighbors]
            except:
                self.neighbors_to_nodes[potenfsetneigh] = set([start])
                self.neighbors_to_nodes[new_neighbors].remove(start)
                if self.neighbors_to_nodes[new_neighbors] == set():
                    del self.neighbors_to_nodes[new_neighbors]
        else:
            self.neighbors_to_nodes[frozenset([end])] = set([start])
            

    def remove_edge(self, start, end):
        """Remove the edge from `start` to `end`."""
        if start in list_of_nodes and end in list_of_nodes:
            #new_ntn = self.neighbors_to_nodes
            for neighbors in self.neighbors_to_nodes:
                nodes = self.neighbors_to_nodes[neighbors]
                if start in nodes and end in neighbors:
                    nodes.remove(start)            
        
        else:
            raise LookupError



#-----------------------Helpers------------------------------------------------------------------


def label_possibilities(node_label, label_list, items = [[]]):
    """Makes a list of every possibly node combination that fits the labels of the pattern without
        checking the edges"""
    if label_list == []:
        return items
    else:
        new_items = []
        for item in items:
            for node in node_label:
                if (node_label[node] == label_list[0]) or (label_list[0]) == "*":
                    item_copy = item.copy()
                    item_copy.append(node)
                    new_items.append(item_copy)
        return label_possibilities(node_label, label_list[1:], new_items)


def test_possib(possibility, pattern, neighbor_dict):
    for i in range(len(pattern)):
        for j in pattern[i][1]:
            if possibility[j] not in neighbor_dict[possibility[i]]:
                return False
    return True

def test_possib_c(possibility, pattern, neighbor_dict):
    for i in range(len(pattern)):      
        for j in pattern[i][1]:
            test = False
            for neighbors in neighbor_dict:
                if possibility[i] in neighbors[1] and possibility[j] in neighbors[0]:
                    test = True
                    break
            if test == False:
                return False
    return True

    


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
    for node in stations: 
        graph.label_dict[node] = stations[node]
    (gift_to_build, elf_count) = build_blank_gift_dict(gift_labels) #create a blank output with all the gift types
    for gift in gift_labels: #add to the blank the buildings connected to the candy
        for pair in graph.query([(gift, [1]),("*", [])]):      
            gift_to_build[gift].append(graph.label_dict[pair[1]])

    inv = dict([[tuple(v),k] for k,v in gift_to_build.items()]) #invert a dictionary of the list of candy and buildings
    for buildings in inv: #implement label rules for elf delivery
        #print(buildings)
        for label in set(buildings):
            if buildings.count(label) >= k:
                elf_count[inv[buildings]] += 1
    return elf_count

def build_blank_gift_dict(gift_labels):
    gift_to_build = {}
    elf_count = {}
    for gift in gift_labels:
        gift_to_build[gift] = []
        elf_count[gift] = 0
    return (gift_to_build, elf_count)



if __name__ == '__main__':
    # Put code here that you want to execute when lab.py is run from the
    # command line, e.g. small test cases.
    for i,j in enumerate([[1, 3], [2], [0], []]):
        print((i,j))
    n = {1: 'a', 2:'a', 3:'b', 4:'c', 5:'d', 6:'d', 7:'c'}
    l = ['a','d','c']
    print(label_possibilities(n,l))
    pass


