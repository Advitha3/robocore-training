#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <iomanip>

using namespace std;

// 1. Define the Node Structure
struct Node {
    int x, y;
    double g, h;
    Node* parent;

    Node(int x, int y, double g, double h, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}

    double f() const {
        return g + h;
    }
};

// Custom comparator for the priority queue (Open List)
// We want the node with the lowest f() value to be at the top.
struct CompareNode {
    bool operator()(const Node* a, const Node* b) const {
        // If f values are equal, tie-break using h value (prefer nodes closer to goal)
        if (a->f() == b->f()) {
            return a->h > b->h; 
        }
        return a->f() > b->f();
    }
};

// 2. Helper Functions
double calculate_euclidean(int x1, int y1, int x2, int y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt((dx * dx) + (dy * dy));
}

bool is_valid(int x, int y, int rows, int cols, const vector<vector<int>>& grid) {
    return (x >= 0 && x < cols && y >= 0 && y < rows && grid[y][x] == 0);
}

// 3. The A* Algorithm
vector<Node*> astar(const vector<vector<int>>& grid, int start_x, int start_y, int goal_x, int goal_y, int& nodes_explored) {
    int rows = grid.size();
    int cols = grid[0].size();
    nodes_explored = 0;

    // The Open List (nodes to evaluate)
    priority_queue<Node*, vector<Node*>, CompareNode> open_list;

    // The Closed List (nodes already evaluated)
    // We use a 2D boolean array for fast lookup
    vector<vector<bool>> closed_list(rows, vector<bool>(cols, false));
    
    // To keep track of the best 'g' cost for each cell to prevent redundant paths
    vector<vector<double>> g_costs(rows, vector<double>(cols, INFINITY));

    // Create the start node
    double start_h = calculate_euclidean(start_x, start_y, goal_x, goal_y);
    Node* start_node = new Node(start_x, start_y, 0.0, start_h);
    
    open_list.push(start_node);
    g_costs[start_y][start_x] = 0.0;

    // 8-Directional movement (dx, dy, cost)
    // Up, Down, Left, Right cost 1.0
    // Diagonals cost sqrt(2) ~ 1.414
    vector<tuple<int, int, double>> directions = {
        {0, -1, 1.0}, {0, 1, 1.0}, {-1, 0, 1.0}, {1, 0, 1.0}, // Cardinal
        {-1, -1, 1.414}, {1, -1, 1.414}, {-1, 1, 1.414}, {1, 1, 1.414} // Diagonal
    };

    while (!open_list.empty()) {
        // Get the node with the lowest f()
        Node* current = open_list.top();
        open_list.pop();

        // If we reached the goal, reconstruct the path
        if (current->x == goal_x && current->y == goal_y) {
            vector<Node*> path;
            Node* temp = current;
            while (temp != nullptr) {
                path.push_back(temp);
                temp = temp->parent;
            }
            reverse(path.begin(), path.end());
            return path;
        }

        // If we've already evaluated this cell with a better or equal cost, skip
        if (closed_list[current->y][current->x]) {
            continue;
        }

        // Mark current node as evaluated
        closed_list[current->y][current->x] = true;
        nodes_explored++;

        // Explore neighbors
        for (const auto& dir : directions) {
            int new_x = current->x + get<0>(dir);
            int new_y = current->y + get<1>(dir);
            double move_cost = get<2>(dir);

            if (is_valid(new_x, new_y, rows, cols, grid) && !closed_list[new_y][new_x]) {
                double new_g = current->g + move_cost;
                
                // If we found a cheaper path to this neighbor
                if (new_g < g_costs[new_y][new_x]) {
                    g_costs[new_y][new_x] = new_g;
                    double new_h = calculate_euclidean(new_x, new_y, goal_x, goal_y);
                    Node* neighbor = new Node(new_x, new_y, new_g, new_h, current);
                    open_list.push(neighbor);
                }
            }
        }
    }

    // No path found
    return {}; 
}

// 4. Dynamic Obstacle Check
bool is_path_blocked(
    const vector<vector<int>>& grid, 
    const vector<pair<int,int>>& path, 
    int obstacle_x, int obstacle_y) 
{
    // (Note: 'grid' is passed to match standard planner API signatures, 
    // though we only need to check the 'path' vector to see if we hit the obstacle)
    for (size_t i = 0; i < path.size(); ++i) {
        if (path[i].first == obstacle_x && path[i].second == obstacle_y) {
            cout << "Replanning required at step " << i << "\n";
            return true;
        }
    }
    cout << "Path still valid\n";
    return false;
}

int main() {
    // 5. Initialize Map
    vector<vector<int>> grid = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 1, 1, 1, 1, 1, 0, 0, 0},
        {0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };

    int start_x = 0;
    int start_y = 0;
    int goal_x = 9;
    int goal_y = 9;
    int nodes_explored = 0;

    // 5. Run A*
    vector<Node*> path = astar(grid, start_x, start_y, goal_x, goal_y, nodes_explored);

    // 6. Print Results
    if (!path.empty()) {
        // Create a display grid
        vector<vector<char>> display_grid(grid.size(), vector<char>(grid[0].size(), '.'));
        
        // Add walls
        for (size_t y = 0; y < grid.size(); ++y) {
            for (size_t x = 0; x < grid[y].size(); ++x) {
                if (grid[y][x] == 1) {
                    display_grid[y][x] = '#';
                }
            }
        }

        // Add path
        for (const Node* n : path) {
            display_grid[n->y][n->x] = '*';
        }

        // Print grid
        for (const auto& row : display_grid) {
            for (char c : row) {
                cout << c << " ";
            }
            cout << endl;
        }

        cout << fixed << setprecision(2);
        cout << "\nPath cost: " << path.back()->g << endl;
        cout << "Nodes explored: " << nodes_explored << endl;
        
        // 1. Convert vector<Node*> to vector<pair<int, int>> for our checker function
        vector<pair<int, int>> path_pairs;
        for (const Node* n : path) {
            path_pairs.push_back({n->x, n->y});
        }
        
        // --- DYNAMIC REPLANNING ---
        cout << "\n--- Dynamic Obstacle Replanning ---" << endl;
        cout << "Obstacle detected at (1, 3)!" << endl;
        
        // Check if the obstacle is on our path
        bool blocked = is_path_blocked(grid, path_pairs, 1, 3);
        
        if (blocked) {
            cout << "\nInitiating replan..." << endl;
            
            // 2. Update the map! (Remember: grid[y][x])
            grid[3][1] = 1;
            
            // 3. Run A* again on the updated grid
            int new_nodes_explored = 0;
            vector<Node*> new_path = astar(grid, start_x, start_y, goal_x, goal_y, new_nodes_explored);
            
            // 4. Print the NEW detour path
            if (!new_path.empty()) {
                vector<vector<char>> detour_grid(grid.size(), vector<char>(grid[0].size(), '.'));
                
                // Add walls (including our new one)
                for (size_t y = 0; y < grid.size(); ++y) {
                    for (size_t x = 0; x < grid[y].size(); ++x) {
                        if (grid[y][x] == 1) detour_grid[y][x] = '#';
                    }
                }
                
                // Add new path
                for (const Node* n : new_path) {
                    detour_grid[n->y][n->x] = '*';
                }
                
                cout << "\nDetour Grid:" << endl;
                for (const auto& row : detour_grid) {
                    for (char c : row) cout << c << " ";
                    cout << endl;
                }
                
                cout << "\nNew Path cost: " << new_path.back()->g << endl;
                cout << "New Nodes explored: " << new_nodes_explored << endl;
            } else {
                cout << "No valid path found! The robot is completely trapped." << endl;
            }
        }
        cout << "-------------------------------" << endl;

    } else {
        cout << "No path found!" << endl;
    }

    return 0;
}