
    def add_edge(self, start, end):
        """Add a edge from `start` to `end`."""
        if start not in self.list_of_nodes:
            raise LookupError
        if end not in self.list_of_nodes:
            raise LookupError
        new_start = False
        new_neighbors = None
        for neighbors, nodes in self.neighbors_to_nodes.items():
            if start in nodes:
                new_start = True
                new_neighbors = neighbors
                break
        if new_start == False: #if we havent even considered the node, then add it to the neighbors {end}
            self.neighbors_to_nodes[frozenset({end})] = {start}
        else: #ok, so this guy already had a list of neighbors, but now we are updating it!
            setn = set(new_neighbors)
            setn.add(end)
            updated_neighbors = frozenset(h) #this is the new set of neighbors for our start
            if updated_neighbors in self.neighbors_to_nodes: #if this neighbors already existed
                self.neighbors_to_nodes[new_neighbors].remove(start)
                if self.neighbors_to_nodes[new_neighbors] == set():
                    del self.neighbors_to_nodes[new_neighbors]
                self.neighbors_to_nodes[updated_neighbors].add(start)
            else:
                self.neighbors_to_nodes[new_neighbors].remove(start)
                if self.neighbors_to_nodes[new_neighbors] == set():
                    del self.neighbors_to_nodes[new_neighbors]
                self.neighbors_to_nodes[updated_neighbors] = set((start,))
