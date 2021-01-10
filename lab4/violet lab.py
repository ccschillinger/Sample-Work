"""6.009 Lab 4 -- Tent Packing"""

# NO IMPORTS ALLOWED!


# Example bag_list entries:
#      vertical 3x1 bag: { (0,0), (1,0), (2,0) }
#      horizontal 1x3 bag: { (0,0), (0,1), (0,2) }
#      square bag: { (0,0), (0,1), (1,0), (1,1) }
#      L-shaped bag: { (0,0), (1,0), (1,1) }
#      C-shaped bag: { (0,0), (0,1), (1,0), (2,0), (2,1) }
#      reverse-C-shaped bag: { (0,0), (0,1), (1,1), (2,0), (2,1) }


def pack(tent_size, missing_squares, bag_list, max_vacancy):
    """
    Pack a tent with different sleeping bag shapes leaving up to max_vacancy squares open
    :param tent_size: (rows, cols) for tent grid
    :param missing_squares: set of (r, c) tuples giving location of rocks
    :param bag_list: list of sets, each describing a sleeping bag shape
    Each set contains (r, c) tuples enumerating contiguous grid
    squares occupied by the bag, coords are relative to the upper-
    left corner of the bag.  You can assume every bag occupies
    at least the grid (0,0).
    :param max_vacancy: maximum number of non-rock locations which can be unoccupied
    :return:  None if no packing can be found; otherwise a list giving the
    placement and type for each placed bag expressed as a dictionary
    with keys
        "anchor": (r, c) for upper-left corner of bag
        "shape": index of bag on bag list
    """
    #the pack_helper code will fail if the upperleft corner is not a possible spot for a sleeping bag
    ul = set()
    for n in range(max_vacancy+1):
        #see if we can pack the tent 
        result = pack_helper(tent_size,missing_squares.union(ul),bag_list, max_vacancy-n)
        if result != None:
            return result
        #if we can't, there's either no solution or the upperleft corner must be left blank
        else:
            #see if we can find a solution by dictating the ul as a missing square
            ul = {get_ul(tent_size, missing_squares)}
    #if we've exceeded the number of missing squares then there must not be a solution
    return result     
            

def pack_helper(tent_size, missing_squares, bag_list, max_vacancy):
    #calculate how many squares are still open
    num_open = tent_size[0]*tent_size[1]-len(missing_squares)
    #if we meet the vacancy condition, return 
    if num_open<=max_vacancy:
        return []
    #else find the upper leftmost free square
    ul = get_ul(tent_size, missing_squares)
    #if there isn't one then we have no solution
    if ul==None:
        return None
    #go through every available bag
    for bag in range(len(bag_list)):
        bag_shift = set()
        #find the coordinates that bag would take up if it started in the ul spot
        for p in bag_list[bag]:
            bag_shift.add((p[0]+ul[0],p[1]+ul[1]))
        #if those coordinates are in the tent
        if bag_in_bounds(bag_shift, tent_size):
            #and they don't hit a rock
            if len(bag_shift-missing_squares)==len(bag_shift):
                #add the bag to the tent and the pack list
                ms = missing_squares.union(bag_shift)
                result = [{'anchor':ul, 'shape':bag}]
                next_bag = pack(tent_size, ms, bag_list, max_vacancy)
                if next_bag!= None:
                    return result+next_bag
    return None
            
def bag_in_bounds(bag, tent_size):
    for coord in bag:
        if coord[0]>=tent_size[0] or coord[1]>=tent_size[1]:
            return False
    return True
    

def get_all_coords(tent_size):
    coords = list()
    for n in range(tent_size[0]):
        for m in range(tent_size[1]):
            coords.append((n,m))
    return coords

def get_ul(tent_size, missing_squares):
    coords = get_all_coords(tent_size)
    for coord in coords:
        if coord not in missing_squares:
            return coord
    return None
            
def ul_blocked(tent_size, missing_squares):
    ul = get_ul(tent_size, missing_squares)
    square = {(0,0), (0,1), (0,2), (1,0), (1,1), (1,2), (2,0), (2,1), (2, 2)}
    for elem in square:
        if (elem[0]+ul[0], elem[1]+ul[1]) in missing_squares:
            return True
    return False

bag_list = [
    {(0, 0), (1, 0), (2, 0)},  # vertical 3x1 bag
    {(0, 0), (0, 1), (0, 2)},  # horizontal 1x3 bag
    {(0, 0), (0, 1), (1, 0), (1, 1)},  # square bag
    {(0, 0), (1, 0), (1, 1)},  # L-shaped bag
    {(0, 0), (0, 1), (1, 0), (2, 0), (2, 1)},  # C-shaped bag
    {(0, 0), (0, 1), (1, 1), (2, 0), (2, 1)},  # reverse C-shaped bag
]


if __name__ == '__main__':
    # additional code here will be run only when lab.py is invoked directly
    # (not when imported from test.py), so this is a good place to put code
    # used, for example, to generate the results for the online questions.
    pass
