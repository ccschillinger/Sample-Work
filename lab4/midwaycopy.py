"""6.009 Lab 4 -- Tent Packing"""

# NO IMPORTS ALLOWED!


# Example bag_list entries:
#      vertical 3x1 bag: { (0,0), (1,0), (2,0) }
#      horizontal 1x3 bag: { (0,0), (0,1), (0,2) }
#      square bag: { (0,0), (0,1), (1,0), (1,1) }
#      L-shaped bag: { (0,0), (1,0), (1,1) }
#      C-shaped bag: { (0,0), (0,1), (1,0), (2,0), (2,1) }
#      reverse-C-shaped bag: { (0,0), (0,1), (1,1), (2,0), (2,1) }

#def pack(tent_size, missing_squares, bag_list, max_vacancy, output = [], tent = None):
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
    nextbagsquare = set()
    for i in range(max_vacancy+1): 
        result = test_bags(tent_size,missing_squares.union(nextbagsquare),bag_list, max_vacancy-i)
        if result == None:
            nextbagsquare = {next_open_square(tent_size, missing_squares)}
        else:
            return result
    return None    

def test_bags(tent_size, missing_squares, bag_list, max_vacancy):
    returnval = False
    nextbagcoord = next_open_square(tent_size, missing_squares)
    if (tent_size[0]*tent_size[1] - len(missing_squares) <= max_vacancy)== True:
        result = []
        returnval = True
    elif not nextbagcoord:
        result = None
        returnval = True
    else:
        for bag in bag_list:
            if can_place(bag, nextbagcoord, missing_squares, tent_size):
                mscopy = missing_squares.copy()
                output = add_bag(bag, nextbagcoord, mscopy)
                next_pack = test_bags(tent_size, mscopy, bag_list, max_vacancy)
                if next_pack != None:
                    result = output+next_pack
                    returnval = True
    if returnval:
       return result
    else:
        return None
        

def can_place(bag, location, occupied, tent_size):
    '''tests to see if a particular piece can be placed at location'''
    bagcoords = set()
    for square in bag:
        coord = (location[0] + square[0], location[1]+square[1])
        bagcoords.add(coord)
    for coords in bagcoords:
        if (coords in occupied) or (not inside_tent(tent_size, coords)):
            return False
    return True

def get_all_coords(tent_size):
    '''returns a list of all squares in the tent'''
    coordlist = list()
    for j in range(tent_size[1]):
        for i in range(tent_size[0]):
            coordlist.append((i, j))
    return coordlist

def next_open_square(tent_size, missing_squares):
    coords = get_all_coords(tent_size)
    for coord in coords:
        if coord not in missing_squares:
            return coord
    return False

def inside_tent(tent_size, coord):
    if coord[0]<0 or coord[0] >= tent_size[0]:
        return False
    if coord[1]<0 or coord[1] >= tent_size[1]:
        return False
    return True

def add_bag(bag, location, occupied):
    coordlist = list()
    for square in bag:
        coord = (location[0] + square[0], location[1]+square[1])
        coordlist.append(coord)
        occupied.add(coord)
    return [{'anchor': location, 'shape': bag_list.index(bag)}]
  
    
            




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
    #print(pack((5,5), {(1,1),(1,3),(3,1)}, bag_list, 0))
    occ = set()
    print(pack((4,4), occ, bag_list, 0))
    pass










































##    nextbagcoord = next_open_square(tent_size, missing_squares)
##    if (tent_size[0]*tent_size[1] - len(missing_squares) <= max_vacancy)== True:
##        
##        return output
##    else:
##        for bag in bag_list:
##            if can_place(bag, nextbagcoord, missing_squares, tent_size):
##                new_output = output.copy()
##                nextbag = {"anchor": nextbagcoord, "shape": bag_list.index(bag)}
##                new_output.append(nextbag)
##                new_missing_squares = missing_squares.copy()
##                #print(new_missing_squares)
##                add_bag(bag, nextbagcoord, new_missing_squares)
##                #print(new_missing_squares)
##                return pack(tent_size, new_missing_squares, bag_list, max_vacancy, new_output)
##        missing_squares.add(nextbagcoord)
##        return pack(tent_size, missing_squares, bag_list, max_vacancy, output)






##    occupied = missing_squares.copy()
##    if tent == None:
##        tent = get_all_coords(tent_size)
##    if (len(tent) - len(missing_squares) <= max_vacancy)== True:
##       return output
##
##    for location in tent:
##        if location not in occupied:
##            backtrack =0
##            for bag in bag_list:
##                if can_place(bag, location, occupied, tent_size):
##                    added_coords = add_bag(bag, location, occupied, output)
##                    next_run = pack(tent_size, occupied, bag_list, max_vacancy, output, tent)
##                    if next_run == None:
##                        output.pop()
##                        for coord in added_coords:
##                            occupied.remove(coord)
##                        backtrack = backtrack+1
##                    else:
##                        return output
##                    
####            if backtrack == len(bag_list):
####                max_vacency = max_vacency-1
####                if max_vacency<0:
####                    return None
##    return None
