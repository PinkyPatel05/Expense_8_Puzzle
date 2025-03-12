import sys
import copy
import heapq
import time
from datetime import datetime
from collections import deque


class PuzzleState:
    #Represents a node of the 8-puzzle.
    def __init__(self, grid, parent=None, move=None, cost=0, depth=0):
        self.grid = grid  
        self.parent = parent  
        self.move = move or "Start" 
        self.cost = cost
        self.depth = depth  
        self.f_value = 0

        # Find the initial position of the 0 blank space
        for i in range(3):
            for j in range(3):
                if self.grid[i][j] == 0:
                    self.blank_pos = (i, j)
                    break

    def __eq__(self, other):
        return self.grid == other.grid

    def __hash__(self):
        return hash(str(self.grid))

    def __lt__(self, other):
        return self.cost < other.cost

    # str representation for trace file format
    def trace_string(self):
        node_str = f"< node = {self.grid}, move = {{{self.move}}} g(n) = {self.cost}, d = {self.depth}, f(n) = {self.f_value}, Parent = Pointer to {{{self.parent_string()}}} >"
        return node_str

    def parent_string(self):
        if self.parent is None:
            return "None"
        return self.parent.trace_string()

    # all moves: up, down, left, right
    def get_possible_moves(self):
        i, j = self.blank_pos
        moves = []

        # up move 
        if i > 0:
            moves.append(("Move {} Down".format(self.grid[i - 1][j]), (i - 1, j)))

        # down move
        if i < 2:
            moves.append(("Move {} Up".format(self.grid[i + 1][j]), (i + 1, j)))

        # left move
        if j > 0:
            moves.append(("Move {} Right".format(self.grid[i][j - 1]), (i, j - 1)))

        # right move
        if j < 2:
            moves.append(("Move {} Left".format(self.grid[i][j + 1]), (i, j + 1)))

        return moves

    # next node generates
    def get_next_node(self, move):
        move_name, (i, j) = move
        new_grid = copy.deepcopy(self.grid)

        # move tile and update blank position
        tile_value = new_grid[i][j]
        new_grid[i][j] = 0
        new_grid[self.blank_pos[0]][self.blank_pos[1]] = tile_value

        # new node with updated cost
        new_cost = self.cost + tile_value
        return PuzzleState(new_grid, self, move_name, new_cost, self.depth + 1)

    # if current node is the goal node or not
    def is_goal(self, target_node):
        return self.grid == target_node.grid

    #Str representation of the node
    def __str__(self):
        res = ""
        for row in self.grid:
            res += " ".join(str(x) for x in row) + "\n"
        return res

# manhattan distance heuristic for A* & Greedy search
def manhattan_distance(node, target_node):
    distance = 0
    for i in range(3):
        for j in range(3):
            if node.grid[i][j] != 0:
    
                for i2 in range(3):
                    for j2 in range(3):
                        if target_node.grid[i2][j2] == node.grid[i][j]:
                            # manhattan distance * tile value
                            tile_value = node.grid[i][j]
                            distance += (abs(i - i2) + abs(j - j2)) * tile_value
                            break
    return distance


# Reads a puzzle node from a file
def read_puzzle_file(filename):
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            row = list(map(int, line.strip().split()))
            grid.append(row)
    return PuzzleState(grid)

# message to the trace file.
def write_trace(file, message):
    file.write(message + "\n")

# returns the path from initial node to the current node.
def get_solution_path(node):
    path = []
    current = node
    while current.parent:
        path.append(current.move)
        current = current.parent
    path.reverse()
    return path

# Breadth-First Search BFS
def bfs(start_node, target_node, dump_flag):
    nodes_popped = 0
    nodes_expanded = 0
    nodes_generated = 1
    max_fringe_size = 1

    trace_file = None
    if dump_flag:
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        trace_file = open(f"Trace-BFS-{timestamp}.txt", 'w')
        write_trace(trace_file, f"Command-Line Arguments : {sys.argv[1:]}")
        write_trace(trace_file, f"Method Selected: BFS")
        write_trace(trace_file, f"Running BFS")

    fringe = deque([start_node])
    closed_set = set()

    while fringe:
    
        max_fringe_size = max(max_fringe_size, len(fringe))
        node = fringe.popleft()
        nodes_popped += 1

        if node.is_goal(target_node):
            solution_path = get_solution_path(node)
            if dump_flag:
                trace_file.close()
            return {
                "nodes_popped": nodes_popped,
                "nodes_expanded": nodes_expanded,
                "nodes_generated": nodes_generated,
                "max_fringe_size": max_fringe_size,
                "solution_depth": node.depth,
                "solution_cost": node.cost,
                "solution_path": solution_path
            }

        node_hash = hash(node)
        if node_hash in closed_set:
            continue
        closed_set.add(node_hash)

        if dump_flag:
            write_trace(trace_file, f"Generating successors to {node.trace_string()}:")

        nodes_expanded += 1
        successors = []

        for move in node.get_possible_moves():
            next_node = node.get_next_node(move)
            next_node_hash = hash(next_node)

            if next_node_hash not in closed_set:
                fringe.append(next_node)
                successors.append(next_node)
                nodes_generated += 1

        if dump_flag and successors:
            write_trace(trace_file, f"\t{len(successors)} successors generated")
            write_trace(trace_file, f"\tClosed: {[node.grid for node in [node]]}")

            fringe_str = "\tFringe: [\n"
            for i, s in enumerate(fringe):
                fringe_str += f"\t\t{s.trace_string()}"
                if i < len(fringe) - 1:
                    fringe_str += "\n"
            fringe_str += "]"
            write_trace(trace_file, fringe_str)


    if dump_flag:
        trace_file.close()

    return {
        "nodes_popped": nodes_popped,
        "nodes_expanded": nodes_expanded,
        "nodes_generated": nodes_generated,
        "max_fringe_size": max_fringe_size,
        "solution_depth": -1,
        "solution_cost": -1,
        "solution_path": None
    }

# Uniform Cost Search UCS
def ucs(start_node, target_node, dump_flag):
    nodes_popped = 0
    nodes_expanded = 0
    nodes_generated = 1
    max_fringe_size = 1

    trace_file = None
    if dump_flag:
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        trace_file = open(f"Trace-UCS-{timestamp}.txt", 'w')
        write_trace(trace_file, f"Command-Line Arguments : {sys.argv[1:]}")
        write_trace(trace_file, f"Method Selected: UCS")
        write_trace(trace_file, f"Running UCS")

    fringe = []
    heapq.heappush(fringe, (start_node.cost, id(start_node), start_node))
    closed_set = set()

    while fringe:
        max_fringe_size = max(max_fringe_size, len(fringe))
        _, _, node = heapq.heappop(fringe)
        nodes_popped += 1

        if node.is_goal(target_node):
            solution_path = get_solution_path(node)
            if dump_flag:
                trace_file.close()
            return {
                "nodes_popped": nodes_popped,
                "nodes_expanded": nodes_expanded,
                "nodes_generated": nodes_generated,
                "max_fringe_size": max_fringe_size,
                "solution_depth": node.depth,
                "solution_cost": node.cost,
                "solution_path": solution_path
            }

        node_hash = hash(node)
        if node_hash in closed_set:
            continue
        closed_set.add(node_hash)

        if dump_flag:
            write_trace(trace_file, f"Generating successors to {node.trace_string()}:")

        nodes_expanded += 1
        successors = []

        for move in node.get_possible_moves():
            next_node = node.get_next_node(move)
            next_node_hash = hash(next_node)

            if next_node_hash not in closed_set:
                heapq.heappush(fringe, (next_node.cost, id(next_node), next_node))
                successors.append(next_node)
                nodes_generated += 1

        if dump_flag and successors:
            write_trace(trace_file, f"\t{len(successors)} successors generated")
            write_trace(trace_file, f"\tClosed: {[node.grid for node in [node]]}")

            fringe_str = "\tFringe: [\n"
            for i, (_, _, s) in enumerate(sorted(fringe)):
                fringe_str += f"\t\t{s.trace_string()}"
                if i < len(fringe) - 1:
                    fringe_str += "\n"
            fringe_str += "]"
            write_trace(trace_file, fringe_str)

    if dump_flag:
        trace_file.close()

    return {
        "nodes_popped": nodes_popped,
        "nodes_expanded": nodes_expanded,
        "nodes_generated": nodes_generated,
        "max_fringe_size": max_fringe_size,
        "solution_depth": -1,
        "solution_cost": -1,
        "solution_path": None
    }

# Depth-First Search DFS
def dfs(start_node, target_node, dump_flag):
    nodes_popped = 0
    nodes_expanded = 0
    nodes_generated = 1
    max_fringe_size = 1

    trace_file = None
    if dump_flag:
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        trace_file = open(f"Trace-DFS-{timestamp}.txt", 'w')
        write_trace(trace_file, f"Command-Line Arguments : {sys.argv[1:]}")
        write_trace(trace_file, f"Method Selected: DFS")
        write_trace(trace_file, f"Running DFS")

    fringe = [start_node]
    closed_set = set()

    while fringe:
        max_fringe_size = max(max_fringe_size, len(fringe))

        node = fringe.pop()
        nodes_popped += 1

        if node.is_goal(target_node):
            solution_path = get_solution_path(node)
            if dump_flag:
                trace_file.close()
            return {
                "nodes_popped": nodes_popped,
                "nodes_expanded": nodes_expanded,
                "nodes_generated": nodes_generated,
                "max_fringe_size": max_fringe_size,
                "solution_depth": node.depth,
                "solution_cost": node.cost,
                "solution_path": solution_path
            }

        node_hash = hash(node)
        if node_hash in closed_set:
            continue
        closed_set.add(node_hash)

        if dump_flag:
            write_trace(trace_file, f"Generating successors to {node.trace_string()}:")

        nodes_expanded += 1
        successors = []

        moves = node.get_possible_moves()
        moves.reverse()

        for move in moves:
            next_node = node.get_next_node(move)
            next_node_hash = hash(next_node)

            if next_node_hash not in closed_set:
                fringe.append(next_node)
                successors.append(next_node)
                nodes_generated += 1

        if dump_flag and successors:
            write_trace(trace_file, f"\t{len(successors)} successors generated")
            write_trace(trace_file, f"\tClosed: {[node.grid for node in [node]]}")

            fringe_str = "\tFringe: [\n"
            for i, s in enumerate(fringe):
                fringe_str += f"\t\t{s.trace_string()}"
                if i < len(fringe) - 1:
                    fringe_str += "\n"
            fringe_str += "]"
            write_trace(trace_file, fringe_str)

    if dump_flag:
        trace_file.close()

    return {
        "nodes_popped": nodes_popped,
        "nodes_expanded": nodes_expanded,
        "nodes_generated": nodes_generated,
        "max_fringe_size": max_fringe_size,
        "solution_depth": -1,
        "solution_cost": -1,
        "solution_path": None
    }

# Depth-Limited Search DLS
def dls(start_node, target_node, depth_limit, dump_flag):
    nodes_popped = 0
    nodes_expanded = 0
    nodes_generated = 1
    max_fringe_size = 1

    trace_file = None
    if dump_flag:
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        trace_file = open(f"Trace-DLS-({depth_limit})-{timestamp}.txt", 'w')
        write_trace(trace_file, f"Command-Line Arguments : {sys.argv[1:]}")
        write_trace(trace_file, f"Method Selected: DLS with depth limit {depth_limit}")
        write_trace(trace_file, f"Running DLS with depth limit {depth_limit}")

    fringe = [start_node]
    closed_set = set()

    while fringe:
        max_fringe_size = max(max_fringe_size, len(fringe))
        node = fringe.pop()
        nodes_popped += 1

        if node.is_goal(target_node):
            solution_path = get_solution_path(node)
            if dump_flag:
                trace_file.close()
            return {
                "nodes_popped": nodes_popped,
                "nodes_expanded": nodes_expanded,
                "nodes_generated": nodes_generated,
                "max_fringe_size": max_fringe_size,
                "solution_depth": node.depth,
                "solution_cost": node.cost,
                "solution_path": solution_path
            }

        # depth limit
        if node.depth >= depth_limit:
            if dump_flag:
                write_trace(trace_file, f"Reached depth limit at depth {node.depth}")
            continue

        node_hash = hash(node)
        closed_key = (node_hash, node.depth)
        if closed_key in closed_set:
            continue
        closed_set.add(closed_key)

        if dump_flag:
            write_trace(trace_file, f"Generating successors to {node.trace_string()}:")

        nodes_expanded += 1
        successors = []

        moves = node.get_possible_moves()
        moves.reverse()

        for move in moves:
            next_node = node.get_next_node(move)
            next_node_hash = hash(next_node)
            next_closed_key = (next_node_hash, next_node.depth)

            if next_closed_key not in closed_set:
                fringe.append(next_node)
                successors.append(next_node)
                nodes_generated += 1

        if dump_flag and successors:
            write_trace(trace_file, f"\t{len(successors)} successors generated")
            write_trace(trace_file, f"\tClosed: {[node.grid for node in [node]]}")

            fringe_str = "\tFringe: [\n"
            for i, s in enumerate(fringe):
                fringe_str += f"\t\t{s.trace_string()}"
                if i < len(fringe) - 1:
                    fringe_str += "\n"
            fringe_str += "]"
            write_trace(trace_file, fringe_str)

    if dump_flag:
        trace_file.close()

    return {
        "nodes_popped": nodes_popped,
        "nodes_expanded": nodes_expanded,
        "nodes_generated": nodes_generated,
        "max_fringe_size": max_fringe_size,
        "solution_depth": -1,
        "solution_cost": -1,
        "solution_path": None
    }

# Iterative Deepening Search IDS
def ids(start_node, target_node, dump_flag):
    total_nodes_popped = 0
    total_nodes_expanded = 0
    total_nodes_generated = 0
    max_fringe_size = 0

    trace_file = None
    if dump_flag:
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        trace_file = open(f"Trace-IDS-{timestamp}.txt", 'w')
        write_trace(trace_file, f"Command-Line Arguments : {sys.argv[1:]}")
        write_trace(trace_file, f"Method Selected: IDS")
        write_trace(trace_file, f"Running IDS")

    depth_limit = 0
    while True:
        if dump_flag:
            write_trace(trace_file, f"\nStarting IDS with depth limit: {depth_limit}")

        res = dls(start_node, target_node, depth_limit, False)

        total_nodes_popped += res["nodes_popped"]
        total_nodes_expanded += res["nodes_expanded"]
        total_nodes_generated += res["nodes_generated"]
        max_fringe_size = max(max_fringe_size, res["max_fringe_size"])

        if dump_flag:
            write_trace(trace_file, f"Completed depth {depth_limit}: "
                                    f"Expanded {res['nodes_expanded']} nodes, "
                                    f"Generated {res['nodes_generated']} nodes")

        if res["solution_path"]:
            if dump_flag:
                write_trace(trace_file, f"Solution found at depth {res['solution_depth']}")
                trace_file.close()

            return {
                "nodes_popped": total_nodes_popped,
                "nodes_expanded": total_nodes_expanded,
                "nodes_generated": total_nodes_generated,
                "max_fringe_size": max_fringe_size,
                "solution_depth": res["solution_depth"],
                "solution_cost": res["solution_cost"],
                "solution_path": res["solution_path"]
            }

        if depth_limit > 30:  
            if dump_flag:
                write_trace(trace_file, "Reached maximum depth limit, stopping search.")
                trace_file.close()

            return {
                "nodes_popped": total_nodes_popped,
                "nodes_expanded": total_nodes_expanded,
                "nodes_generated": total_nodes_generated,
                "max_fringe_size": max_fringe_size,
                "solution_depth": -1,
                "solution_cost": -1,
                "solution_path": None
            }

        depth_limit += 1

# Greedy Search 
def greedy(start_node, target_node, dump_flag):
    nodes_popped = 0
    nodes_expanded = 0
    nodes_generated = 1
    max_fringe_size = 1

    trace_file = None
    if dump_flag:
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        trace_file = open(f"Trace-Greedy-{timestamp}.txt", 'w')
        write_trace(trace_file, f"Command-Line Arguments : {sys.argv[1:]}")
        write_trace(trace_file, f"Method Selected: Greedy")
        write_trace(trace_file, f"Running Greedy Search")

    fringe = []
    h_value = manhattan_distance(start_node, target_node)
    start_node.f_value = h_value
    heapq.heappush(fringe, (h_value, id(start_node), start_node))
    closed_set = set()

    while fringe:
        max_fringe_size = max(max_fringe_size, len(fringe))
        _, _, node = heapq.heappop(fringe)
        nodes_popped += 1

        if node.is_goal(target_node):
            solution_path = get_solution_path(node)
            if dump_flag:
                trace_file.close()
            return {
                "nodes_popped": nodes_popped,
                "nodes_expanded": nodes_expanded,
                "nodes_generated": nodes_generated,
                "max_fringe_size": max_fringe_size,
                "solution_depth": node.depth,
                "solution_cost": node.cost,
                "solution_path": solution_path
            }

        node_hash = hash(node)
        if node_hash in closed_set:
            continue
        closed_set.add(node_hash)

        if dump_flag:
            write_trace(trace_file, f"Generating successors to {node.trace_string()}:")

        nodes_expanded += 1
        successors = []

        for move in node.get_possible_moves():
            next_node = node.get_next_node(move)
            next_node_hash = hash(next_node)

            if next_node_hash not in closed_set:
                h_value = manhattan_distance(next_node, target_node)
                next_node.f_value = h_value  
                heapq.heappush(fringe, (h_value, id(next_node), next_node))
                successors.append(next_node)
                nodes_generated += 1

        if dump_flag and successors:
            write_trace(trace_file, f"\t{len(successors)} successors generated")
            write_trace(trace_file, f"\tClosed: {[node.grid for node in [node]]}")

            fringe_str = "\tFringe: [\n"
            for i, (_, _, s) in enumerate(sorted(fringe)):
                fringe_str += f"\t\t{s.trace_string()}"
                if i < len(fringe) - 1:
                    fringe_str += "\n"
            fringe_str += "]"
            write_trace(trace_file, fringe_str)

    if dump_flag:
        trace_file.close()

    return {
        "nodes_popped": nodes_popped,
        "nodes_expanded": nodes_expanded,
        "nodes_generated": nodes_generated,
        "max_fringe_size": max_fringe_size,
        "solution_depth": -1,
        "solution_cost": -1,
        "solution_path": None
    }

# A* search using manhattan distance heuristic with updated trace formate
def a_star(start_node, target_node, dump_flag):
    nodes_popped = 0
    nodes_expanded = 0
    nodes_generated = 1
    max_fringe_size = 1

    trace_file = None
    if dump_flag:
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        trace_file = open(f"Trace-ASTAR-{timestamp}.txt", 'w')
        write_trace(trace_file, f"Command-Line Arguments : {sys.argv[1:]}")
        write_trace(trace_file, f"Method Selected: a*")
        write_trace(trace_file, f"Running a*")

    fringe = []
    h_value = manhattan_distance(start_node, target_node)
    start_node.f_value = start_node.cost + h_value
    heapq.heappush(fringe, (start_node.f_value, id(start_node), start_node))
    closed_set = set()


    while fringe:
        _, _, node = heapq.heappop(fringe)
        nodes_popped += 1

        if node.is_goal(target_node):
            solution_path = get_solution_path(node)
            if dump_flag:
                trace_file.close()
            return {
                "nodes_popped": nodes_popped,
                "nodes_expanded": nodes_expanded,
                "nodes_generated": nodes_generated,
                "max_fringe_size": max_fringe_size,
                "solution_depth": node.depth,
                "solution_cost": node.cost,
                "solution_path": solution_path
            }

        node_hash = hash(node)
        if node_hash in closed_set:
            continue
        closed_set.add(node_hash)

        if dump_flag:
            write_trace(trace_file, f"Generating successors to {node.trace_string()}:")

        nodes_expanded += 1
        moves = node.get_possible_moves()
        successors = []

        for move in moves:
            next_node = node.get_next_node(move)
            next_node_hash = hash(next_node)

            if next_node_hash not in closed_set:
                h_value = manhattan_distance(next_node, target_node)
                next_node.f_value = next_node.cost + h_value
                heapq.heappush(fringe, (next_node.f_value, id(next_node), next_node))
                successors.append(next_node)
                nodes_generated += 1

        max_fringe_size = max(max_fringe_size, len(fringe))


        if dump_flag and successors:
            write_trace(trace_file, f"\t{len(successors)} successors generated")
            write_trace(trace_file, f"\tClosed: {[node.grid for node in [node]]}")

            fringe_str = "\tFringe: [\n"
            for i, (_, _, s) in enumerate(sorted(fringe)):
                fringe_str += f"\t\t{s.trace_string()}"
                if i < len(fringe) - 1:
                    fringe_str += "\n"
            fringe_str += "]"
            write_trace(trace_file, fringe_str)

    if dump_flag:
        trace_file.close()
        
    return {
        "nodes_popped": nodes_popped,
        "nodes_expanded": nodes_expanded,
        "nodes_generated": nodes_generated,
        "max_fringe_size": max_fringe_size,
        "solution_depth": -1,
        "solution_cost": -1,
        "solution_path": None
    }


# Main function
def main():
    if len(sys.argv) < 3:
        print("Usage: expense_8_puzzle.py <start-file> <goal-file> <method> <dump-flag>")
        sys.exit(1)

    start_file = sys.argv[1]
    goal_file = sys.argv[2]

    # default method is A*
    method = "a*" if len(sys.argv) < 4 else sys.argv[3]

    # default dump flag is False
    dump_flag = False if len(sys.argv) < 5 else sys.argv[4].lower() == "true"

    try:
        start_node = read_puzzle_file(start_file)
        target_node = read_puzzle_file(goal_file)

        res = None

        if method == "bfs":
            res = bfs(start_node, target_node, dump_flag)
        elif method == "ucs":
            res = ucs(start_node, target_node, dump_flag)
        elif method == "dfs":
            res = dfs(start_node, target_node, dump_flag)
        elif method == "dls":
            depth_limit = int(input("Enter depth limit: "))
            res = dls(start_node, target_node, depth_limit, dump_flag)
        elif method == "ids":
            res = ids(start_node, target_node, dump_flag)
        elif method == "greedy":
            res = greedy(start_node, target_node, dump_flag)
        elif method == "a*":
            res = a_star(start_node, target_node, dump_flag)
        else:
            sys.exit(1)

        if res["solution_path"]:
            print(f"___Output of {method.upper()} Algorithm___")
            # print(f"Algorithm: {method.upper()}")
            print(f"Nodes Popped: {res['nodes_popped']}")
            print(f"Nodes Expanded: {res['nodes_expanded']}")
            print(f"Nodes Generated: {res['nodes_generated']}")
            print(f"Max Fringe Size: {res['max_fringe_size']}")
            print(f"Solution Found at depth {res['solution_depth']} with cost of {res['solution_cost']}.")
            print("Steps:")
            for step in res["solution_path"]:
                print(step)
        else:
            print("No solution found.")

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

