import lab, json, traceback, time
from importlib import reload
reload(lab)  # this forces the student code to be reloaded when page is refreshed

# These functions are required by the UI
def query(d):
    try:
        pattern = eval(d["query"])
        for _tuple in pattern:
            assert isinstance(_tuple[0], str)
            assert isinstance(_tuple[1], list)
            for match in _tuple[1]:
                assert isinstance(match, int) and 0 <= match < len(pattern)
    except:
        return "Error: Query pattern is invalid"

    try:
        gf = lab.GraphFactory(eval("lab." + d["graphClass"]))
    except:
        return "Error: Invalid graph class or Graph Factory failed to init"

    try:
        graph = gf.from_list(adj_list, labels)
    except:
        return "Error: Graph Factor failed to create graph"

    try:
        return graph.query(pattern), pattern
    except:
        return "Error: Query Function had an error"


# State that is used by both ui and test code
adj_list = None
labels = None
adj_dict = None


## Initialization
def init():
    global adj_list
    global labels
    global adj_dict
    with open('resources/ui/ui_graph.json', 'r') as f:
        graph_dict = json.load(f)
    adj_list = []
    labels = {}
    adj_dict = {}
    for i in range(len(graph_dict)):
        adj_list.append(graph_dict[str(i)][1])
        labels[i] = graph_dict[str(i)][0]
        adj_dict[i] = graph_dict[str(i)][1]

init()
