import copy
import math
import random

'''
 IMPORTANT: Read through the code before beginning implementation!
 Your solution should fill in the various "TODO" items within this starter code.
'''
g_CYCLE_TIME = .100


# Default parameters will create a 4x4 grid to test with
g_MAP_SIZE_X = 2. # 2m wide
g_MAP_SIZE_Y = 1.5 # 1.5m tall
g_MAP_RESOLUTION_X = 0.5 # Each col represents 50cm
g_MAP_RESOLUTION_Y = 0.375 # Each row represents 37.5cm
g_NUM_X_CELLS = int(g_MAP_SIZE_X // g_MAP_RESOLUTION_X) # Number of columns in the grid map
g_NUM_Y_CELLS = int(g_MAP_SIZE_Y // g_MAP_RESOLUTION_Y) # Number of rows in the grid map

# Map from Lab 4: values of 0 indicate free space, 1 indicates occupied space
g_WORLD_MAP = [0] * g_NUM_Y_CELLS*g_NUM_X_CELLS # Initialize graph (grid) as array

# Source and Destination (I,J) grid coordinates
g_dest_coordinates = (5,5)
g_src_coordinates = (0,0)


def create_test_map(map_array):
  # Takes an array representing a map of the world, copies it, and adds simulated obstacles
  num_cells = len(map_array)
  new_map = copy.copy(map_array)
  # Add obstacles to up to sqrt(n) vertices of the map
  for i in range(int(math.sqrt(len(map_array)))):
    random_cell = random.randint(0, num_cells-1)
    map_array[random_cell] = 1

  return map_array

def vertex_index_to_ij(vertex_index):
  '''
  vertex_index: unique ID of graph vertex to be converted into grid coordinates
  Returns COL, ROW coordinates in 2D grid
  '''
  global g_NUM_X_CELLS
  return vertex_index % g_NUM_X_CELLS, vertex_index // g_NUM_X_CELLS

def ij_to_vertex_index(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns integer 'vertex index'
  '''
  global g_NUM_X_CELLS
  return j*g_NUM_X_CELLS + i


def ij_coordinates_to_xy_coordinates(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return (i+0.5)*g_MAP_RESOLUTION_X, (j+0.5)*g_MAP_RESOLUTION_Y

def xy_coordinates_to_ij_coordinates(x,y):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return int(i // g_MAP_RESOLUTION_X), int(j // g_MAP_RESOLUTION_Y)

# **********************************
# *      Core Dijkstra Functions   *
# **********************************

def get_travel_cost(vertex_source, vertex_dest):
  # Returns the cost of moving from vertex_source (int) to vertex_dest (int)
  # INSTRUCTIONS:
  global g_NUM_X_CELLS,g_NUM_Y_CELLS, g_WORLD_MAP
  '''
      This function should return 1 if:
        vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)

      This function should return 1000 if:
        vertex_source corresponds to (i,j) coordinates outside the map
        vertex_dest corresponds to (i,j) coordinates outside the map
        vertex_source and vertex_dest are not adjacent to each other (i.e., more than 1 move away from each other)
  '''
  if(vertex_source not in range(len(g_WORLD_MAP)) or vertex_dest not in range(len(g_WORLD_MAP))):
    return 1000

  j_source, i_source = vertex_index_to_ij(vertex_source)
  j_dest, i_dest = vertex_index_to_ij(vertex_dest)

  if (i_dest not in range(g_NUM_Y_CELLS)) or (j_dest not in range(g_NUM_X_CELLS)):
    return 1000

  diff = abs(i_source-i_dest)+abs(j_source-j_dest)+g_WORLD_MAP[vertex_source]+g_WORLD_MAP[vertex_dest] # Should this be vertex_dest?


  return 1000 if diff>1 else 1

def find_neighbors(vertex):
  global g_NUM_Y_CELLS,g_NUM_X_CELLS
  i,j = vertex_index_to_ij(vertex)
  return [ij_to_vertex_index(i-1,j), ij_to_vertex_index(i+1,j), ij_to_vertex_index(i,j-1), ij_to_vertex_index(i,j+1)]

def run_dijkstra(source_vertex):
    '''
    source_vertex: vertex index to find all paths back to
    returns: 'prev' array from a completed Dijkstra's algorithm run

    Function to return an array of ints corresponding to the 'prev' variable in Dijkstra's algorithm
    The 'prev' array stores the next vertex on the best path back to source_vertex.
    Thus, the returned array prev can be treated as a lookup table:  prev[vertex_index] = next vertex index on the path back to source_vertex
    '''
    global g_NUM_X_CELLS, g_NUM_Y_CELLS, g_WORLD_MAP

    # Array mapping vertex_index to distance of shortest path from vertex_index to source_vertex.
    dist = [1000] * g_NUM_X_CELLS * g_NUM_Y_CELLS
    dist[source_vertex] = 0
    # Queue for identifying which vertices are up to still be explored:
    # Will contain tuples of (vertex_index, cost), sorted such that the min cost is first to be extracted (explore cheapest/most promising vertices first)
    Q_cost = [(i, dist[i]) for i in range(len(g_WORLD_MAP))]

    # Array of ints for storing the next step (vertex_index) on the shortest path back to source_vertex for each vertex in the graph
    prev = [-1] *g_NUM_X_CELLS*g_NUM_Y_CELLS

    # Insert your Dijkstra's code here. Don't forget to initialize Q_cost properly!

    while(len(Q_cost) > 0):
        costs = [y[1] for y in Q_cost]
        i = min(costs)
        u = Q_cost[costs.index(i)]
        Q_cost.pop(costs.index(i))
        for v in find_neighbors(u[0]):
            if(v in range(len(g_WORLD_MAP))):
                verticies = [y[0] for y in Q_cost]
                alt = dist[u[0]] + get_travel_cost(u[0], v)
                if (alt < dist[v]):
                    dist[v] = alt
                    prev[v] = u[0]
                    index_v = verticies.index(v)
                    hold = (Q_cost[index_v][0], dist[v])
                    Q_cost[index_v] = hold
    # #code to print cost array:
    # for i in range(g_NUM_Y_CELLS-1, -1, -1):
    #     for j in range(g_NUM_X_CELLS):
    #         vertex = ij_to_vertex_index(j,i)
    #         print('[{}]'.format(dist[ij_to_vertex_index(j,i)]), end=" ")
    #     print('')
    # Return results of algorithm run
    return prev


def reconstruct_path(prev, source_vertex, dest_vertex):
  '''
  Given a populated 'prev' array, a source vertex_index, and destination vertex_index,
  allocate and return an integer array populated with the path from source to destination.
  The first entry of your path should be source_vertex and the last entry should be the dest_vertex.
  If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the
  path from dest to source, return an empty list.
  '''
  final_path = []

  # TODO: Insert your code here
  # Empty prev array
  if(len(prev) == 0):
    return final_path

  val = dest_vertex
  while val != source_vertex:
    final_path.insert(0, val)
    val = prev[val]
  final_path.insert(0,val)
  return final_path

def render_map(map_array):
  global g_WORLD_MAP,g_NUM_Y_CELLS,g_NUM_X_CELLS

  '''
  TODO-
    Display the map in the following format:
    Use " . " for free grid cells
    Use "[ ]" for occupied grid cells

    Example:
    For g_WORLD_MAP = [0, 0, 1, 0,
                       0, 1, 1, 0,
                       0, 0, 0, 0,
                       0, 0, 0, 0]
    There are obstacles at (I,J) coordinates: [ (2,0), (1,1), (2,1) ]
    The map should render as:
      .  .  .  .
      .  .  .  .
      . [ ][ ] .
      .  . [ ] .


    Make sure to display your map so that I,J coordinate (0,0) is in the bottom left.
    (To do this, you'll probably want to iterate from row 'J-1' to '0')
  '''
  for i in range(g_NUM_Y_CELLS-1, -1, -1):
    for j in range(g_NUM_X_CELLS):
        vertex = ij_to_vertex_index(j,i)
        if g_WORLD_MAP[vertex]==1:
            print('[ ]', end=" ")
        else:
            print(' . ', end=" ")
    print('')

def generateRandomVertex(source = -1):
    global g_WORLD_MAP
    num_cells = len(g_WORLD_MAP)
    random_cell = random.randint(0, num_cells-1)
    while(g_WORLD_MAP[random_cell] == 1 or random_cell==source):
        random_cell = random.randint(0, num_cells-1)
    return random_cell


def main():
    global g_WORLD_MAP,g_NUM_X_CELLS,g_NUM_Y_CELLS

    # TODO: Initialize a grid map to use for your test -- you may use create_test_map for this, or manually set one up with obstacles
    # g_WORLD_MAP = [0, 0, 1, 0,
    #                 0, 1, 1, 0,
    #                 0, 0, 0, 0,
    #                 0, 0, 0, 0]
    # g_NUM_X_CELLS = 4
    # g_NUM_Y_CELLS = 4
    # Use render_map to render your initialized obstacle map
    g_WORLD_MAP = create_test_map(g_WORLD_MAP)
    render_map(g_WORLD_MAP)
    # TODO: Find a path from the (I,J) coordinate pair in g_src_coordinates to the one in g_dest_coordinates using run_dijkstra and reconstruct_path
    source = generateRandomVertex()
    target = generateRandomVertex(source)
    j_s,i_s = vertex_index_to_ij(source)
    j_g, i_g = vertex_index_to_ij(target)
    prev = run_dijkstra(source)
    path = reconstruct_path(prev, source, target)
    '''
    TODO-
    Display the final path in the following format:
    Source: (0,0)
    Goal: (3,1)
    0 -> 1 -> 2 -> 6 -> 7
    '''
    print('Source: ', '({},{})'.format(j_s,i_s), ', Goal: ', '({},{})'.format(j_g,i_g))
    for i in range(len(path)-1):
        print(str(path[i])+' ->', end=" ")
    print(path[-1])



if __name__ == "__main__":
  main()
