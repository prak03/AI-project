import time
import heapq

# Defining the goal state of the 8-puzzle
goal_state = (1, 2, 3, 4, 5, 6, 7, 8, 0)

# These are the possible moves for each block
moves = {
    0: (1, 3),
    1: (0, 2, 4),
    2: (1, 5),
    3: (0, 4, 6),
    4: (1, 3, 5, 7),
    5: (2, 4, 8),
    6: (3, 7),
    7: (4, 6, 8),
    8: (5, 7)
}
# We now define a function for manhattan distance calculation 

def manhattan_distance(state):
    dist = 0
    for i in range(3):
        for j in range(3):
            if state[3*i+j] != 0:
                dist += abs(i - (state[3*i+j]-1)//3) + abs(j - (state[3*i+j]-1) % 3)
    return dist
# this function is created for calculating the a* search with manhattan distance
def a_star_search(initial_state):
    # We first create a priority queue to store the search nodes
    queue = [(manhattan_distance(initial_state), 0, initial_state)]
    # We Create a dictionary to store the g-score for each state
    g_scores = {initial_state: 0}
    # We Create a dictionary to store the parent state for each state
    parents = {initial_state: None}
    # We Create a set to store the visited states
    visited = set()
    # Initializing the node count
    nodes_explored = 0
    # We Loop until the queue is empty
    while queue:
        # Get the node with the lowest f-score
        _, cost, state = heapq.heappop(queue)
        # Check if the state is the goal state
        if state == goal_state:
            path = []
            while state:
                path.append(state)
                state = parents[state]
            return cost, path[::-1], nodes_explored
        # Check if the state has been visited
        if state in visited:
            continue
        # Mark the state as visited
        visited.add(state)
        # Increment the node count
        nodes_explored += 1
        # Generate the child nodes
        for move in moves[state.index(0)]:
            new_state = list(state)
            new_state[state.index(
                0)], new_state[move] = new_state[move], new_state[state.index(0)]
            new_state = tuple(new_state)
            # Calculate the cost and heuristic of the new node
            new_cost = g_scores[state] + 1
            new_heuristic = manhattan_distance(new_state)
            # Update the g-score and parent for the new state
            if new_state not in g_scores or new_cost < g_scores[new_state]:
                g_scores[new_state] = new_cost
                parents[new_state] = state
                # Add the new node to the queue
                priority = new_cost + new_heuristic
                heapq.heappush(queue, (priority, new_cost, new_state))
    # If we reach here, we did not find the goal state, so we return none 
    return None, None, nodes_explored



#function for uniform cost search
def uniform_cost_search(initial_state):
    # We Create a priority queue to store the search nodes
    queue = [(0, initial_state)]
    # We Create a dictionary to store the g-score for each state
    g_scores = {initial_state: 0}
    # We Create a dictionary to store the parent state for each state
    parents = {initial_state: None}
    # We Create a set to store the visited states
    visited = set()
    # We Initialize the node count
    nodes_explored = 0
    # We Increment the node count for the initial state
    nodes_explored += 1
    # Loop until the queue is empty
    while queue:
        # Get the node with the lowest g-score
        _, state = heapq.heappop(queue)
        # Check if the state is the goal state
        if state == goal_state:
            path = []
            while state:
                path.append(state)
                state = parents[state]
            return len(path) - 1, path[::-1], nodes_explored
        # Check if the state has been visited
        if state in visited:
            continue
        # Mark the state as visited
        visited.add(state)
        # Generate the child nodes
        for move in moves[state.index(0)]:
            new_state = list(state)
            new_state[state.index(
                0)], new_state[move] = new_state[move], new_state[state.index(0)]
            new_state = tuple(new_state)
            # Calculate the cost of the new node
            new_cost = g_scores[state] + 1
            # Update the g-score and parent for the new state
            if new_state not in g_scores or new_cost < g_scores[new_state]:
                g_scores[new_state] = new_cost
                parents[new_state] = state
                # Increment the node count for each newly generated child node
                nodes_explored += 1
                # Add the new node to the queue
                heapq.heappush(queue, (new_cost, new_state))
    # If we reach here, we did not find the goal state
    return None, None, nodes_explored


# this function is to calculate the hueristic while using the misplaced tiles
def misplaced_tile(state):
    misplaced = 0
    for i in range(len(state)):
        if state[i] != goal_state[i]:
            misplaced += 1
    return misplaced

# function for a* search with misplaced tile
def a_star_search_misp(initial_state):
    # Create a priority queue to store the search nodes
    queue = [(misplaced_tile(initial_state), 0, initial_state)]
    # Create a dictionary to store the g-score for each state
    g_scores = {initial_state: 0}
    # Create a dictionary to store the parent state for each state
    parents = {initial_state: None}
    # Create a set to store the visited states
    visited = set()
    # Initialize the node count
    nodes_explored = 0
    # Loop until the queue is empty
    while queue:
        # Get the node with the lowest f-score
        _, cost, state = heapq.heappop(queue)
        # Check if the state is the goal state
        if state == goal_state:
            path = []
            while state:
                path.append(state)
                state = parents[state]
            return cost, path[::-1], nodes_explored
        # Check if the state has been visited
        if state in visited:
            continue
        # Mark the state as visited
        visited.add(state)
        # Increment the node count
        nodes_explored += 1
        # Generate the child nodes
        # Generate the child nodes
        for move in moves[state.index(0)]:
            new_state = list(state)
            new_state[state.index(
                0)], new_state[move] = new_state[move], new_state[state.index(0)]
            new_state = tuple(new_state)
            # Calculate the cost and heuristic of the new node
            new_cost = g_scores[state] + 1
            new_heuristic = misplaced_tile(new_state)
            # Update the g-score and parent for the new state
            if new_state not in g_scores or new_cost < g_scores[new_state]:
                g_scores[new_state] = new_cost
                parents[new_state] = state
                # Add the new node to the queue
                priority = new_cost + new_heuristic
                heapq.heappush(queue, (priority, new_cost, new_state))
    # If we get here, we did not find the goal state
    return None, None, nodes_explored

# this is used to know the direction
def get_move_direction(curr_state, next_state):
    curr_row, curr_col = divmod(curr_state.index(0), 3)
    next_row, next_col = divmod(next_state.index(0), 3)

    if curr_row == next_row and curr_col + 1 == next_col:
        return "Right"
    elif curr_row == next_row and curr_col - 1 == next_col:
        return "Left"
    elif curr_row + 1 == next_row and curr_col == next_col:
        return "Down"
    elif curr_row - 1 == next_row and curr_col == next_col:
        return "Up"
    else:
        return "Invalid move"

# The file starts executng here
print(" This is my 8-puzzle solver")
while True:
    print("\n")
    print("Type '1' to use a default puzzle or '2' to give a puzzle as an input")
    if int(input())==1:
        initial_state = (1,2,3,5,0,6,4,7,8)
    else:
        print("Give your input puzzle and represent the blank with a '0'.")
        a=list(map(int,input().split()))
        initial_state=tuple(a)

    print("Select the algorithm you would like to perform onn the given input puzzle :")
    print("(1) Uniform cost search \n")
    print("(2) A* with Misplaced tile hueristic \n")
    print("(3) A* with Manhattan distance hueristic \n")
    print("Any other number for quitting the game")

    
    b = int(input())
    if b == 1:
        start_time2 = time.time()
        cost2, path2, ans2 = uniform_cost_search(initial_state)
        end_time2 = time.time()
        if cost2 is None:
            print("The goal state could not be reached.")
        else:
            print("The minimum number of moves to reach the goal state is:", cost2)
            print("The path to the goal state is:")
            for i in range(len(path2) - 1):
                state = path2[i]
                next_state = path2[i + 1]
                
                print("Move:", get_move_direction(state, next_state))
                print(state[:3])
                print(state[3:6])
                print(state[6:])
                
                print()
        print("Nodes explored in uniform cost search:", ans2)
        print("Time taken for uniform cost search:", (end_time2 - start_time2)*1000, "ms")
    elif b == 2:
        start_time3 = time.time()
        cost3, path3, ans3 = a_star_search_misp(initial_state)
        end_time3 = time.time()
        if cost3 is None:
            print("The goal state could not be reached.")
        else:
            print("The minimum number of moves to reach the goal state is:", cost3)
            print("The path to the goal state is:")
            for i in range(len(path3) - 1):
                state = path3[i]
                next_state = path3[i + 1]
                
                print("Move:", get_move_direction(state, next_state))
                print(state[:3])
                print(state[3:6])
                print(state[6:])
                
                print()
        print("Nodes explored in A* with misplaced tile heuristic:", ans3)
        print("Time taken for A* with misplaced tile heuristic:", (end_time3 - start_time3)*1000, "ms")
    elif b == 3:
        start_time1 = time.time()
        cost1, path1, ans1 = a_star_search(initial_state)
        end_time1 = time.time()
        if cost1 is None:
            print("The goal state could not be reached.")
        else:
            print("The minimum number of moves to reach the goal state is:", cost1)
            print("The path to the goal state is:")
            for i in range(len(path1) - 1):
                state = path1[i]
                next_state = path1[i + 1]
                
                print("Move:", get_move_direction(state, next_state))
                print(state[:3])
                print(state[3:6])
                print(state[6:])
                
                print()
        print("Nodes explored in A* with Manhattan distance heuristic:", ans1)
        print("Time taken for A* with Manhattan distance heuristic:", (end_time1 - start_time1) * 1000, "ms")
    else:
        print("Thank you for playing")
        break