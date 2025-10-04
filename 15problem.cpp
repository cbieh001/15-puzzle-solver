#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <map>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <tuple>
#include <deque>
#include <random> 
#include <ctime>

using namespace std;

// max num of nodes before terminating (prevents infinite loops)
const int maxNumNodes = 100000; 
const int gridSideLength = 4; // grid size = 4x4
const int gridSize = gridSideLength * gridSideLength;

/**
 * The struct Node represents a node in a 4x4 grid puzzle with various attributes such as grid state,
 * blank tile position, costs, parent index, and previous move.
 * @property gridState - The `gridState` property represents a 4x4 grid stored as a vector of integers.
 * Each integer in the vector represents a cell in the grid, with 0 typically representing an empty
 * cell.
 * @property {int} blankTilePos - The `blankTilePos` property in the `Node` struct represents the index
 * of the blank tile (0) in the 4x4 grid stored as a vector in the `gridState` property.
 * @property {int} startNodeCost - The `startNodeCost` property in the `Node` struct represents the
 * cost from the start node, which is essentially the depth of the current node in the search tree. It
 * keeps track of how many steps have been taken to reach the current state from the initial state.
 * @property {int} heuristicCost - The `heuristicCost` property in the `Node` struct represents the
 * estimated cost from the current state to the goal state based on a heuristic function. This
 * heuristic function provides an estimate of how close the current state is to the goal state, helping
 * guide the search algorithm towards a solution efficiently.
 * @property {int} costSum - The `costSum` property in the `Node` struct represents the sum of the
 * `startNodeCost` and the `heuristicCost`. It is used to evaluate the total cost of a node in a search
 * algorithm, where `startNodeCost` represents the cost from the start node (depth
 * @property {int} indexOfParent - The `indexOfParent` property in the `Node` struct represents the
 * index of the parent node in a vector of nodes. This property is used in algorithms like A* search to
 * keep track of the parent node of a particular node in order to reconstruct the path from the start
 * node to the current node
 * @property {char} prevMove - The `prevMove` property in the `Node` struct represents the move that
 * led to the current state. It stores a character indicating the direction of the move, such as 'U'
 * for up, 'D' for down, 'L' for left, and 'R' for right.
 */
struct Node 
{
    vector<int> gridState; // 4x4 grid as vector
    int blankTilePos;     // index of blank tile (0-15)
    int startNodeCost;        // cost from the start node (depth)
    int heuristicCost;        // heuristic cost (estimated cost to goal)
    int costSum;        // startNodeCost + heuristicCost
    int indexOfParent;  // index in the sumNodes vector
    char prevMove;         // move that led to current state

    // initial node constructor
    Node(const vector<int>& s, int p_idx = -1, char m = 'S') 
        : gridState(s), startNodeCost(0), heuristicCost(0), costSum(0), indexOfParent(p_idx), prevMove(m) 
        {
        auto currState = find(gridState.begin(), gridState.end(), 0);
        blankTilePos = distance(gridState.begin(), currState);
    }

    // default node constructor
    Node() : blankTilePos(-1), startNodeCost(0), heuristicCost(0), costSum(0), indexOfParent(-1), prevMove(' ') 
    {

    }
};

// helper for A* priority queue: stores pStartNodeCost and the index of the node
/**
 * The struct informedHelper represents a data structure with cost sum, heuristic cost, and index
 * fields.
 * @property {int} costSum - The `costSum` property in the `informedHelper` struct represents the total
 * cost accumulated so far in a path, typically from the start node to the current node.
 * @property {int} heuristicCost - The `heuristicCost` property in the `informedHelper` struct
 * represents the estimated cost from the current node to the goal node based on a heuristic function.
 * It is used in informed search algorithms like A* to guide the search towards the goal efficiently by
 * considering both the actual cost from the start
 * @property {int} index - The `index` property in the `informedHelper` struct represents the index of
 * the node or element being processed. It is used to keep track of the position or identifier of the
 * node within a data structure or algorithm.
 */
struct informedHelper 
{
    int costSum;
    int heuristicCost;
    int index;

    informedHelper(int pStartNodeCost, int pHeuristicCost, int pIndex) : costSum(pStartNodeCost), heuristicCost(pHeuristicCost), index(pIndex) {}
};

// helper comparator for A* search: prioritize lower F-cost
/**
 * The struct informedHelperComp defines a comparison function for informedHelper objects based on
 * costSum and heuristicCost.
 * 
 */
struct informedHelperComp 
{
    bool operator()(const informedHelper& a, const informedHelper& b) 
    {
        if (a.costSum != b.costSum) {
            return a.costSum > b.costSum; 
        }
        return a.heuristicCost > b.heuristicCost; // tie-breaker -> prefers lower heuristic cost
    }
};

// function to get the goal position
/**
 * The function findGoalPos returns the row and column indices of a given value tile in a grid.
 * 
 * @param valTile The `valTile` parameter represents the value of a tile in a grid.
 * 
 * @return A pair of integers representing the row and column indices of the goal position for a given
 * value tile in a grid. If the value tile is 0, the goal position is at the bottom right corner of the
 * grid.
 */
pair<int, int> findGoalPos(int valTile) 
{
    if (valTile == 0) 
    {
        return {gridSideLength - 1, gridSideLength - 1};
    }
    int goal_index = valTile - 1;
    return {goal_index / gridSideLength, goal_index % gridSideLength};
}

// h1(x)
/**
 * The function calculates the number of misplaced tiles in a given puzzle state.
 * 
 * @param pState It looks like the function `calcH1` is calculating the number of misplaced tiles in a
 * puzzle state represented by the vector `pState`. The function compares the values in the vector with
 * the expected values (1 to gridSize) and counts the number of tiles that are not in their correct
 * positions.
 * 
 * @return The function `calcH1` is returning the number of misplaced tiles in the given state
 * represented by the vector `pState`.
 */
int calcH1(const vector<int>& pState) 
{
    int misplacedTiles = 0;
    for (int i = 0; i < gridSize; ++i) 
    {
        if (pState[i] != 0 && pState[i] != i + 1) 
        {
            misplacedTiles++;
        }
    }
    return misplacedTiles;
}

// h2(x)
/**
 * The function calculates the Manhattan distance heuristic for the 15-puzzle game given the current
 * state.
 * 
 * @param pState The `pState` parameter is a constant reference to a vector of integers. It represents
 * the current state of a puzzle grid where each integer value corresponds to a tile in the grid. The
 * function `calcH2` calculates the total Manhattan distance of each tile from its goal position in the
 * grid.
 * 
 * @return The function `calcH2` calculates the total Manhattan distance between the current position
 * of each tile in the puzzle represented by the `pState` vector and its goal position. The function
 * iterates through each tile in the puzzle, calculates the distance between the current position and
 * the goal position for each tile, and accumulates the total distance. Finally, the function returns
 * the total distance as an integer value
 */
int calcH2(const vector<int>& pState) 
{
    int totalDistance = 0;
    for (int i = 0; i < gridSize; ++i) 
    {
        int currTile = pState[i];
        if (currTile == 0) continue; // ignores blank tile

        // current position
        int currRow = i / gridSideLength;
        int currCol = i % gridSideLength;

        // goal position
        pair<int, int> goalPos = findGoalPos(currTile);
        int goalRow = goalPos.first;
        int goalCol = goalPos.second;

        totalDistance += abs(currRow - goalRow) + abs(currCol - goalCol);
    }
    return totalDistance;
}

// h3(x)
/**
 * The function calculates the heuristic value H3 for a given state in a sliding puzzle game by summing
 * the Manhattan distances of each tile to its goal position with different weights for specific tiles.
 * 
 * @param pState The function `calcH3` calculates the heuristic value for a given state represented by
 * the vector `pState`. The heuristic value is calculated based on the weighted Manhattan distance
 * between the current position of each tile and its goal position.
 * 
 * @return The function `calcH3` is returning an integer value that is the rounded total weighted
 * distance calculated based on the input vector `pState`.
 */
int calcH3(const vector<int>& pState) 
{
    double totalWeightedDistance = 0;
    for (int i = 0; i < gridSize; ++i) 
    {
        int currTile = pState[i];
        if (currTile == 0) continue; 

        // current position
        int currRow = i / gridSideLength;
        int currCol = i % gridSideLength;

        // goal position
        pair<int, int> goalPos = findGoalPos(currTile);
        int goalRow = goalPos.first;
        int goalCol = goalPos.second;

        int currDistance = abs(currRow - goalRow) + abs(currCol - goalCol);
        double currWeight = 1.0;
        
        if (currTile >= 13 && currTile <= 15) 
        {
            currWeight = 1.5;
        }

        totalWeightedDistance += currDistance * currWeight;
    }

    return static_cast<int>(round(totalWeightedDistance));
}

// check if current state is goal state
/**
 * The function `isGoalState` checks if a given state matches the goal state for a 15-puzzle game.
 * 
 * @param pState The `pState` parameter is a constant reference to a vector of integers. It represents
 * the current state of a puzzle or game board where each integer value corresponds to a tile or cell
 * in the grid. The goal state for the puzzle is to have the numbers 1 to 15 in order followed
 * 
 * @return The function `isGoalState` returns a boolean value. It returns `true` if the input vector
 * `pState` represents the goal state of a puzzle, which is defined as having the values 1 to 15 in
 * order followed by a 0. If the input vector does not match this goal state, the function returns
 * `false`.
 */
bool isGoalState(const vector<int>& pState) 
{
    // Goal state: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0
    for (int i = 0; i < gridSize - 1; ++i) 
    {
        if (pState[i] != i + 1) 
        {
            return false;
        }
    }
    return pState[gridSize - 1] == 0;
}

// get state as string
/**
 * The function `stateToString` converts a vector of integers into a string representation with each
 * integer separated by a comma.
 * 
 * @param pState The `pState` parameter is a vector of integers representing the current state of a
 * puzzle or game board. The function `stateToString` takes this vector and converts it into a string
 * representation where each integer in the vector is separated by a comma.
 * 
 * @return The function `stateToString` returns a string that represents the contents of the vector
 * `pState` by concatenating each integer element with a comma separator.
 */
string stateToString(const vector<int>& pState) 
{
    stringstream ss;
    for (int currTile : pState) 
    {
        ss << currTile << ",";
    }
    return ss.str();
}

// generate next possible states by moving blank tile
/**
 * The function `generateNextState` generates possible successor nodes for a given node in a grid based
 * on possible moves and a heuristic function.
 * 
 * @param currNode The `currNode` parameter represents the current state of the puzzle grid. It
 * contains information such as the grid state, position of the blank tile, the cost to reach this node
 * from the start, the heuristic cost, and the total cost sum.
 * @param heuristicFunction The `heuristicFunction` parameter is a pointer to a function that takes a
 * `vector<int>` as input and returns an integer. This function is used to calculate the heuristic cost
 * for a given state in the context of generating the next state in a search algorithm. The heuristic
 * cost is an estimate of
 * 
 * @return The function `generateNextState` returns a vector of `Node` objects, which represent the
 * possible successor states from the current node `currNode` based on the available moves (UP, DOWN,
 * LEFT, RIGHT) and the constraints of the grid. Each `Node` object in the vector represents a
 * potential next state with updated grid configuration, move character, costs, and position of the
 * blank tile
 */
vector<Node> generateNextState(const Node& currNode, int (*heuristicFunction)(const vector<int>&)) 
{
    vector<Node> nodeSuccessors;
    int tempRow = currNode.blankTilePos / gridSideLength; // current row
    int tempCol = currNode.blankTilePos % gridSideLength; // current col

    // possible moves
    vector<tuple<int, int, char>> possibleMoves = 
    {
        {-1, 0, 'U'}, // UP
        { 1, 0, 'D'}, // DOWN
        { 0, -1, 'L'}, // LEFT
        { 0, 1, 'R'}  // RIGHT
    };

    for (const auto& currMoveInfo : possibleMoves) 
    {
        int currRow = get<0>(currMoveInfo);
        int currCol = get<1>(currMoveInfo);
        char moveCharacter = get<2>(currMoveInfo);

        int newRow = tempRow + currRow; // new row
        int newCol = tempCol + currCol; // new col
        int newBlankPositipon = newRow * gridSideLength + newCol; // new index

        // checks bounds
        if (newRow >= 0 && newRow < gridSideLength && newCol >= 0 && newCol < gridSideLength) 
        {
            vector<int> nextState = currNode.gridState;
            swap(nextState[currNode.blankTilePos], nextState[newBlankPositipon]);
            Node nextNode(nextState, -1, moveCharacter);
            nextNode.blankTilePos = newBlankPositipon;
            nextNode.startNodeCost = currNode.startNodeCost + 1;
            nextNode.heuristicCost = (heuristicFunction) ? heuristicFunction(nextState) : 0; 
            nextNode.costSum = nextNode.startNodeCost + nextNode.heuristicCost;

            nodeSuccessors.push_back(nextNode);
        }
    }
    return nodeSuccessors;
}

// retraces solution path from goal node
/**
 * The function `retracePath` retraces the path from a goal node to the start node in a deque of nodes
 * and calculates the total memory usage in KB based on the number of nodes stored and the memory per
 * node.
 * 
 * @param sumNodes The `sumNodes` parameter is a deque of `Node` objects. It seems to represent a
 * sequence of nodes that have been traversed or processed in some way during an algorithm. Each `Node`
 * object likely contains information such as the index of its parent node, the previous move made to
 * reach
 * @param goalIndex The `goalIndex` parameter represents the index of the goal node in the `sumNodes`
 * deque that you want to retrace the path to. It is used to determine the node from which the path
 * retracing should start.
 * @param memoryPerNode The `memoryPerNode` parameter represents the memory consumed by each node in
 * kilobytes. This function `retracePath` takes a deque of `Node` objects, an integer `goalIndex`, and
 * the memory consumed per node as input. It then retraces the path from the goal node
 * 
 * @return A pair containing a string representing the path from the start node to the goal node and a
 * long long integer representing the total memory usage in kilobytes.
 */
pair<string, long long> retracePath(const deque<Node>& sumNodes, int goalIndex, long long memoryPerNode) 
{
    string currPath = "";
    int currIndex = goalIndex;
    
    // check if goal node index is valid
    if (goalIndex < 0 || goalIndex >= sumNodes.size()) 
    {
        return {"", 0}; 
    }

    // traverse back from goal to start node
    while (sumNodes[currIndex].indexOfParent != -1) 
    {
        currPath += sumNodes[currIndex].prevMove;
        currIndex = sumNodes[currIndex].indexOfParent;
    }

    // reverse path to get moves from start to goal
    reverse(currPath.begin(), currPath.end());

    // memory usage: (total nodes stored * size of node) in KB
    long long totalNodesStored = sumNodes.size();
    long long totalMemoryUsage = (long long)ceil(((double)totalNodesStored * memoryPerNode) / 1024.0);
    
    return {currPath, totalMemoryUsage};
}

/**
 * The function `runSearchAlgorithm` implements various search strategies (BFS, DFS, informed search)
 * to find a solution path in a search space and returns relevant statistics.
 * 
 * @param searchStrategyName The `searchStrategyName` parameter is a string that specifies the type of
 * search algorithm to be used. It can be "BFS" for Breadth-First Search, "DFS" for Depth-First Search,
 * or any other string for an informed search algorithm.
 * @param startNode The `startNode` parameter in the `runSearchAlgorithm` function represents the
 * initial state or node from which the search algorithm will begin its exploration. It contains
 * information about the current state of the problem, such as the grid state, the cost to reach that
 * state from the start node, the heuristic
 * @param heuristicFunction The `heuristicFunction` parameter in the `runSearchAlgorithm` function is a
 * function pointer that takes a `const vector<int>&` as input and returns an `int`. This function is
 * used to calculate the heuristic cost for a given state in the search algorithm. It is optional and
 * can be
 * @param shuffleSuccessors The `shuffleSuccessors` parameter in the `runSearchAlgorithm` function is a
 * boolean flag that determines whether the successors generated during the search algorithm should be
 * shuffled before processing them. If `shuffleSuccessors` is set to `true`, the order of successors
 * will be randomized using a random number generator
 * 
 * @return The function `runSearchAlgorithm` is returning a tuple containing the following elements:
 * 1. A boolean indicating whether a solution was found (`isSolved`).
 * 2. An integer representing the number of nodes expanded during the search.
 * 3. A string representing the path taken to reach the solution.
 * 4. A long long integer representing the total memory used in kilobytes during the search.
 */
tuple<bool, int, string, long long> runSearchAlgorithm(const string& searchStrategyName, Node startNode, int (*heuristicFunction)(const vector<int>&) = nullptr, bool shuffleSuccessors = false) 
{
    
    cout << "\n-----------------------------------------" << endl;
    cout << "Searching strategy: " << searchStrategyName << endl;
    
    auto startTime = chrono::high_resolution_clock::now();

    // deque to store all nodes created; index in deque is official Node ID.
    deque<Node> sumNodes;
    sumNodes.push_back(startNode);
    
    // visited set
    map<string, int> visitedSet; 
    visitedSet[stateToString(startNode.gridState)] = startNode.startNodeCost;

    queue<int> fringeBFS; 
    stack<int> fringeDFS; 
    priority_queue<informedHelper, vector<informedHelper>, informedHelperComp> fringeInformed;
    
    int nodesExpanded = 0;
    bool isSolved = false;
    int goalNodeIndex = -1;
    int currIndex = -1;
    
    // DFS
    bool isDFS = searchStrategyName.find("DFS") != string::npos;

    // BFS
    if (searchStrategyName.find("BFS") != string::npos) 
    {
        fringeBFS.push(0);
    } 
    else if (isDFS) 
    {
        fringeDFS.push(0);
    } 
    else 
    { 
        // informed search
        startNode.heuristicCost = heuristicFunction(startNode.gridState);
        startNode.costSum = startNode.startNodeCost + startNode.heuristicCost;
        sumNodes[0] = startNode; 
        fringeInformed.push(informedHelper(startNode.costSum, startNode.heuristicCost, 0));
    }

    default_random_engine generator(chrono::system_clock::now().time_since_epoch().count());

    // loop to keep program running until user quits
    while (nodesExpanded < maxNumNodes) 
    {
        if (searchStrategyName.find("BFS") != string::npos) 
        {
            if (fringeBFS.empty()) 
            {
                break;
            }
            currIndex = fringeBFS.front();
            fringeBFS.pop();
        } 
        else if (isDFS) 
        {
            if (fringeDFS.empty()) 
            {
                break;
            }
            currIndex = fringeDFS.top();
            fringeDFS.pop();
        } 
        else 
        { 
            if (fringeInformed.empty()) 
            {
                break;
            }
            informedHelper current_pq = fringeInformed.top();
            fringeInformed.pop();
            currIndex = current_pq.index;
            
            if (sumNodes[currIndex].startNodeCost > visitedSet[stateToString(sumNodes[currIndex].gridState)]) 
            {
                continue;
            }
        }

        const Node& currentNode = sumNodes[currIndex];
        nodesExpanded++;

        // checks for goal
        if (isGoalState(currentNode.gridState)) 
        {
            isSolved = true;
            goalNodeIndex = currIndex;
            break;
        }

        // generates successors
        vector<Node> nodeSuccessors = generateNextState(currentNode, heuristicFunction);
        if (shuffleSuccessors) 
        {
            shuffle(nodeSuccessors.begin(), nodeSuccessors.end(), generator);
        }

        for (Node& nextNode : nodeSuccessors) 
        {
            string nextNodeString = stateToString(nextNode.gridState);
            int nextGCost = nextNode.startNodeCost;
            bool isVisited = visitedSet.count(nextNodeString);
            if (!isDFS) 
            {
                if (isVisited && nextGCost >= visitedSet[nextNodeString]) 
                {
                    continue; 
                }
            }
            nextNode.indexOfParent = currIndex;
            sumNodes.push_back(nextNode);
            int nextIndex = sumNodes.size() - 1;
           visitedSet[nextNodeString] = nextGCost;
            if (searchStrategyName.find("BFS") != string::npos) 
            {
                fringeBFS.push(nextIndex);
            } 
            else if (isDFS) 
            {
                // limits DFS depth
                if (nextNode.startNodeCost < 30) 
                {
                    fringeDFS.push(nextIndex);
                }
            } 
            else 
            { 
                fringeInformed.push(informedHelper(nextNode.costSum, nextNode.heuristicCost, nextIndex));
            }
        }
    }


    // -------------------------------------------------
    // Outputs
    // -------------------------------------------------
    auto endTime = chrono::high_resolution_clock::now();
    long long durationInMS = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
    
    // memory usage
    long long memoryPerNode = sizeof(Node);
    long long totalNodesStored = sumNodes.size();
    long long totalMemory = (long long)ceil(((double)totalNodesStored * memoryPerNode) / 1024.0);
    
    // prints stats for this run
    string pathTaken = "";
    auto pathTakenData = retracePath(sumNodes, goalNodeIndex, memoryPerNode);
    pathTaken = pathTakenData.first;
    totalMemory = pathTakenData.second;
    cout << "Moves: " << pathTaken << endl;
    cout << "Number of nodes expanded: " << nodesExpanded << endl;
    cout << "Time taken: " << durationInMS << "ms" << endl;
    cout << "Memory used: " << totalMemory << "kb" << endl;
    
    if (isSolved) 
    {
        cout << "A solution was found" << endl;
    } 
    else 
    {
        if (nodesExpanded >= maxNumNodes) 
        {
            cout << "No solution was found. Node expansion limit was reached (" << maxNumNodes << ")." << endl;
        } 
        else 
        {
            cout << "No solution was found. No path found within the depth/node limit." << endl;
        }
    }
    cout << "-----------------------------------------" << endl;
    
    return {isSolved, nodesExpanded, pathTaken, totalMemory};
}

// -----------------------------------------------------------
// main program
// -----------------------------------------------------------

// prints puzzle state
/**
 * The function `printState` prints the elements of a vector in a grid format.
 * 
 * @param currState It looks like the `printState` function is designed to print the elements of a
 * vector `currState` in a grid format. The function iterates over the elements of `currState` and
 * prints them with a width of 2 characters using `setw(2)`.
 */
void printState(const vector<int>& currState) 
{
    for (int i = 0; i < gridSize; ++i) 
    {
        cout << setw(2) << currState[i] << " ";
        if ((i + 1) % gridSideLength == 0) 
        {
            cout << endl;
        }
    }
}

// validates custom puzzle
/**
 * The function `getUserPuzzle` reads 16 tile values from the user input and returns them as a vector.
 * 
 * @return The function `getUserPuzzle` returns a vector of integers containing the 16 tile values
 * input by the user. If there is an issue with the input (e.g., non-integer input), an empty vector is
 * returned.
 */
vector<int> getUserPuzzle() {
    vector<int> currState(gridSize);
    cout << "Enter 16 tile values (from 0-15; 0 represents a blank tile) separated by spaces:" << endl;
    cout << "Example puzzle (solvable): 1 2 3 4 5 6 7 8 9 10 11 12 13 14 0 15" << endl;
    cout << "Input: ";
    // checks all 16 values are read
    for (int i = 0; i < gridSize; ++i) 
    {
        if (!(cin >> currState[i])) 
        {
            cerr << "Issue with input, try again." << endl;
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            return {}; 
        }
    }
    return currState;
}

// main function
/**
 * The main function implements a 15-Puzzle Solver with options to choose default or custom puzzles and
 * select different search algorithms like Breadth-First Search, Depth-First Search, and A* with
 * various heuristics.
 * 
 * @return The `main` function is returning an integer value of 0, which is a common practice in C++ to
 * indicate successful program execution.
 */
int main() 
{
    // default state once
    const vector<int> defaultInitialState = {1, 0, 2, 4, 5, 7, 3, 8, 9, 6, 11, 12, 13, 10, 14, 15}; 
    string mainChoice;
    
    // main loop for continuous operation
    while (true) 
    {
        cout << "\n\n--- 15-Puzzle Solver Main Menu ---" << endl;
        cout << "Default Puzzle State:" << endl;
        printState(defaultInitialState);

        cout << "\nSelect an option:" << endl;
        cout << "1. Default Puzzle" << endl;
        cout << "2. Custom Puzzle" << endl;
        cout << "q. Quit" << endl;
        cout << "Enter choice (1, 2, q): ";

        if (!(cin >> mainChoice)) 
        {
            cerr << "Input error. Exiting." << endl;
            break;
        }

        // checks for quit
        if (mainChoice == "q" || mainChoice == "Q") 
        {
            cout << "\nGoodbye!" << endl;
            break;
        }

        vector<int> initState;
        if (mainChoice == "2") 
        {
            initState = getUserPuzzle();
        } 
        else if (mainChoice == "1") 
        {
            initState = defaultInitialState;
        } 
        else 
        {
            cout << "\nInvalid selection. Try again." << endl;
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            continue;
        }
        
        if (initState.empty()) 
        {
             cout << "\nFailed to read custom puzzle. Returning to main menu." << endl;
             continue;
        }
        
        cout << "\nStarting Puzzle:" << endl;
        printState(initState);
        
        if (isGoalState(initState)) 
        {
            cout << "\nChosen state is already the goal state. No moves needed. Returning to menu." << endl;
            continue;
        }

        Node startNode(initState);
        
        // --- search algorithm menu ---
        int searchAlgorithmChoice;
        cout << "\nSelect Search Algorithm to Run:" << endl;
        cout << "1. Breadth-First Search (BFS)" << endl;
        cout << "2. Depth-First Search (DFS - limited, random, with retry logic)" << endl;
        cout << "3. Informed Search (A*) using Heuristics" << endl;
        cout << "Enter choice (1-3): ";
        
        if (!(cin >> searchAlgorithmChoice)) 
        {

            cerr << "\nInvalid search input. Returning to main menu." << endl;
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            continue; 
        }

        switch (searchAlgorithmChoice) 
        {
            case 1: 
            {
                runSearchAlgorithm("Breadth-First Search (BFS)", startNode);
                break;
            }
            case 2: 
            {
                const int maxDFSRetries = 3;
                bool isDFSSolved = false;
                
                for (int currAttempt = 1; currAttempt <= maxDFSRetries; ++currAttempt) 
                {
                    cout << "Attempt " << currAttempt << " of " << maxDFSRetries << ": Running DFS" << endl;
                    
                    auto [isSolved, nodesExpanded, currPath, currMemory] = runSearchAlgorithm("Depth-First Search (DFS - limited, random, with retry logic)", startNode, nullptr, true);
                    
                    if (isSolved) 
                    {
                        isDFSSolved = true;
                        cout << "\nA solution was found on attempt " << currAttempt << "!" << endl;
                        cout << "Path: " << currPath << endl;
                        break;
                    }
                    
                    if (currAttempt < maxDFSRetries) 
                    {
                        cout << "Node limit reached/search failed. Retrying with different branch order." << endl;
                    }
                }

                if (!isDFSSolved) 
                {
                    cout << "\nSearch failed." << endl;
                    cout << "Solution not possible within " << maxDFSRetries << " randomized attempts or the fixed node limit of " << maxNumNodes << "." << endl;
                }
                break;
            }
            case 3: 
            {
                int informedChoice;
                cout << "\nSelect Heuristic (pHeuristicCost(x)) for A*:" << endl;
                cout << "1. h1(x) = number of misplaced tiles" << endl;
                cout << "2. h2(x) = sum of the distances of every tile to its goal positions" << endl;
                cout << "3. h3(x) = sum of the distances of every tile to its goal positions (weighted)" << endl;
                cout << "Enter heuristic choice (1-3): ";

                if (!(cin >> informedChoice)) 
                {
                     cerr << "Invalid input. Running A* with h1(x) (number of misplaced tiles)." << endl;
                     informedChoice = 1;
                }

                if (informedChoice == 1) 
                {
                    runSearchAlgorithm("A* - h1(x) number of misplaced tiles", startNode, calcH1);
                } 
                else if (informedChoice == 2) 
                {
                    runSearchAlgorithm("A* - h2(x) sum of the distances of every tile to its goal positions", startNode, calcH2);
                } 
                else if (informedChoice == 3) 
                {
                    runSearchAlgorithm("A* - h3(x) sum of the distances of every tile to its goal positions (weighted)", startNode, calcH3);
                } 
                else 
                {
                    cout << "Invalid heuristic choice. Running A* with h1(x) (number of misplaced tiles)." << endl;
                    runSearchAlgorithm("A* - h2(x) sum of the distances of every tile to its goal positions)", startNode, calcH2);
                }
                break;
            }
            default:
                cout << "Invalid search algorithm choice. Returning to main menu." << endl;
                break;
        }
    }

    return 0;
}
