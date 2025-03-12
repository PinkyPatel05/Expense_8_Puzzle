Artificial Intellegent Project:

Programming language Used:Python 3.x

Expense 8 Puzzle Solver

Overview:
This program solves a modified version of the 8-puzzle classic problem. In this variation, moving a tile costs the value of the number on the tile (e.g., moving tile "6" costs 6 units). The program implements different search algorithms to find the optimal solution path from a start node to a target node.


Key Features: (All search Algorithms)
  BFS (Breadth-First Search)
  UCS (Uniform Cost Search)
  DFS (Depth-First Search)
  DLS (Depth-Limited Search)
  IDS (Iterative Deepening Search)
  Greedy Search (using manhattan distance heuristic)
  A* Search (using manhattan distance heuristic)(default)

Outputs including below Information:
  Nodes Popped
  Nodes Expanded
  Nodes Generated
  Maximum Fringe Size
  Solution Path, Cost, and Depth

Manhattan Distance heuristic is used for greedy and a* search algorithms.
Trace file Feature is Optional. The trace file feature is optional and depends on the status of the <dump_flag>. If method(e.g. bfs true) is true, the trace file is generated.
This programs works on both Windows and macOS/Linux environments.

Requirements:
Python 3.x
Terminal or command prompt to run the script
Required files: expense_8_puzzle.py, start.txt, goal.txt(no external dependencies)

How to Run code on terminal:
1.No installation required. Simply download the expense_8_puzzle.py file to your local machine.
2.Run the command as expense_8_puzzle.py start.txt goal.txt <method> <dump-flag>, Here methods that can be used are(bfs,ucs,dfs,dls,ids,greedy and a*) and dump can be true or false.
3.If <method> and <dump-flag> has no input given it takes default values as a* and false respectively

Parameters:
<start-file>:-> Take path to the file containing the initial state.(required)
<goal-file>:-> Take path to the file containing the goal state.(required)
<method>:-> Search algorithms (default run: a*).(optional)
<dump-flag>:-> true create trace file, false don't create trace file

Below commands used to run the file on terminal:
python3 expense_8_puzzle.py start.txt goal.txt
python3 expense_8_puzzle.py start.txt goal.txt a* true
python3 expense_8_puzzle.py start.txt goal.txt bfs false
python3 expense_8_puzzle.py start.txt goal.txt ucs true
python3 expense_8_puzzle.py start.txt goal.txt dfs true
python3 expense_8_puzzle.py start.txt goal.txt dls true 
python3 expense_8_puzzle.py start.txt goal.txt greedy false 

Implementation Steps of program:
1. In the beginning start state, goal state, method and dump file inputs are taken from command line respectively ex: expense_8_puzzle.py start.txt goal.txt "a*" true.
2. In the PuzzleState class represent the 8-puzzle problem and stores the puzzle grid, parent state, move taken, cost, depth, and heuristic value. It also determines the blank space for 0 position.
3. __eq__(): This constructor used of checking if two puzzle states are equal.
4. __hash__(): This constructor is used to returns a unique hash for the puzzle state.
5. __lt__(): This constructor is defines comparison based on cost (for priority queue use).
6. trace_string(): The function returns a formatted string for the puzzle state.
7. parent_string(): The function returns the parent state in string format.
8. get_possible_moves(): this functions returns all valid moves such as up, down, left, right.
9. get_next_node(): This functions generates the next puzzle state based on a move.
10. is_goal(): This function checks if the current state matches with target node.
11. __str__(): This constructor converts the puzzle state into a readable string.
12. manhattan_distance(): this function calculate the Manhattan distance heuristic which used in A* and Greedy search Algorithms.
13. read_puzzle_file(): this function reads a puzzle configuration from a file.
14. write_trace(): Writes string represent for trace file formate.
15. get_solution_path(): this function retrieves the path from the initial to the goal state.
16. Then after applies search Algorithm which executes the chosen search method such as BFS,UCS,DFS,DLS,IDS,Greedy or A*.
17. In the last write main()function it reads input arguments, loads the puzzle states, and runs the selected search algorithm.

Note: When using the a* option in a shell, you may need to escape or quote it: "a*" or a\*

Manhattan Distance Heuristic Function:
For A* and Greedy search, a modified Manhattan distance heuristic is used that accounts for tile costs:
	For each tile, calculate Manhattan distance × tile value
	Sum these values to get the heuristic value

Trace File (Optional)
If <dump-flag> is set to true, the program generates a trace file named trace-<method>-<timestamp>.txt, containing:
  Contents of the fringe and closed set at each step
  Details of nodes expanded and generated


Input File Format
Each input file (start.txt and goal.txt) should contain a 3x3 grid, where 0 represents the blank tile.

Example (start.txt):
2 3 6
1 0 7
4 8 5

Example (goal.txt):
1 2 3
4 5 6
7 8 0

Example Output:
___Output of A* Algorithm___
Nodes Popped: 60
Nodes Expanded: 59
Nodes Generated: 101
Max Fringe Size: 42
Solution Found at depth 12 with cost of 63.
Steps:
 Move 7 Left
 Move 5 Up
 Move 8 Right
 Move 7 Down
 Move 5 Left
 Move 6 Down
 Move 3 Right
 Move 2 Right
 Move 1 Up
 Move 4 Up
 Move 7 Left
 Move 8 Left

File Directory Structure
8-Puzzle-Solver/
│── expense_8_puzzle.py     # Main python script file
│── start.txt               # Start state file
│── goal.txt                # Goal state file
│── trace_file.txt          # generates trace file(if pass method true)
│── README.txt              # Detailed documentation file 

Conclusion
The expense_8_puzzle.py provides a optimal solution to solving the 8-puzzle problem using different search techniques. The optional trace file feature allows detailed search process analysis, making it a useful tool for understanding and evaluating all search algorithms.

