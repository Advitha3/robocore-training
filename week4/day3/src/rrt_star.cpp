#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <utility>

using namespace std;

// --- Constants ---
const double STEP_SIZE = 5.0;
const int MAX_ITER = 5000;
const double GOAL_THRESHOLD = 5.0;
const double REWIRE_RADIUS = 15.0; // NEW CONSTANT

// --- Structs ---
struct RRTNode {
    double x, y;
    int parent_idx;
    double cost; // NEW: Track distance from root
};

struct Rect {
    double x, y, w, h;
};

// --- Helper Functions ---
double distance(const RRTNode& a, const RRTNode& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

double distance_xy(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

int nearest_node(const vector<RRTNode>& tree, double x, double y) {
    int closest_idx = 0;
    double min_dist = 1e9;
    for (int i = 0; i < tree.size(); i++) {
        double dist = distance_xy(tree[i].x, tree[i].y, x, y);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    return closest_idx;
}

bool collides(double x1, double y1, double x2, double y2, const vector<Rect>& obstacles) {
    int steps = 10;
    for (int i = 0; i <= steps; i++) {
        double t = (double)i / steps;
        double test_x = x1 + t * (x2 - x1);
        double test_y = y1 + t * (y2 - y1);
        
        for (const auto& obs : obstacles) {
            if (test_x >= obs.x && test_x <= obs.x + obs.w &&
                test_y >= obs.y && test_y <= obs.y + obs.h) {
                return true;
            }
        }
    }
    return false;
}

pair<double, double> steer(const RRTNode& from, double to_x, double to_y) {
    double dist = distance_xy(from.x, from.y, to_x, to_y);
    if (dist < STEP_SIZE) {
        return {to_x, to_y};
    }
    double new_x = from.x + ((to_x - from.x) / dist) * STEP_SIZE;
    double new_y = from.y + ((to_y - from.y) / dist) * STEP_SIZE;
    return {new_x, new_y};
}

// Calculate the total physical distance of a reconstructed path
double calculate_path_cost(const vector<RRTNode>& path) {
    double total = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
        total += distance(path[i-1], path[i]);
    }
    return total;
}

// --- Smoothing Function ---
void smooth_path(vector<RRTNode>& path, const vector<Rect>& obstacles) {
    if (path.size() <= 2) return; // Nothing to smooth
    
    for (int k = 0; k < 300; k++) {
        int i = rand() % path.size();
        int j = rand() % path.size();
        
        if (i > j) swap(i, j);
        
        // Only attempt to bridge if there is actually a gap of at least 1 node to skip
        if (j > i + 1) {
            if (!collides(path[i].x, path[i].y, path[j].x, path[j].y, obstacles)) {
                // Erase everything strictly between i and j
                path.erase(path.begin() + i + 1, path.begin() + j);
            }
        }
    }
}

// --- Planners ---

// 1. Standard RRT Algorithm
vector<RRTNode> rrt(RRTNode start, RRTNode goal, const vector<Rect>& obstacles) {
    vector<RRTNode> tree;
    tree.push_back({start.x, start.y, -1, 0.0});
    
    for (int iter = 0; iter < MAX_ITER; iter++) {
        double rand_x, rand_y;
        if (rand() % 100 < 10) { 
            rand_x = goal.x; rand_y = goal.y; 
        } else {
            rand_x = (rand() % 1000) / 10.0;
            rand_y = (rand() % 1000) / 10.0;
        }

        int nearest_idx = nearest_node(tree, rand_x, rand_y);
        RRTNode nearest = tree[nearest_idx];
        
        auto [new_x, new_y] = steer(nearest, rand_x, rand_y);
        
        if (!collides(nearest.x, nearest.y, new_x, new_y, obstacles)) {
            RRTNode new_node = {new_x, new_y, nearest_idx, nearest.cost + distance_xy(nearest.x, nearest.y, new_x, new_y)};
            tree.push_back(new_node);
            
            if (distance_xy(new_x, new_y, goal.x, goal.y) < GOAL_THRESHOLD) {
                vector<RRTNode> path;
                int curr = tree.size() - 1;
                while (curr != -1) {
                    path.push_back(tree[curr]);
                    curr = tree[curr].parent_idx;
                }
                reverse(path.begin(), path.end());
                return path;
            }
        }
    }
    return {}; 
}

// 2. RRT* (Star) Algorithm
vector<RRTNode> rrt_star(RRTNode start, RRTNode goal, const vector<Rect>& obstacles) {
    vector<RRTNode> tree;
    tree.push_back({start.x, start.y, -1, 0.0});
    
    for (int iter = 0; iter < MAX_ITER; iter++) {
        double rand_x = (rand() % 1000) / 10.0;
        double rand_y = (rand() % 1000) / 10.0;
        
        int nearest_idx = nearest_node(tree, rand_x, rand_y);
        RRTNode nearest = tree[nearest_idx];
        
        auto [new_x, new_y] = steer(nearest, rand_x, rand_y);
        
        // MODIFICATION 1: Choose best parent within REWIRE_RADIUS
        if (!collides(nearest.x, nearest.y, new_x, new_y, obstacles)) {
            
            RRTNode q_new = {new_x, new_y, nearest_idx, 0.0}; // parent temporarily nearest
            double min_cost = nearest.cost + distance_xy(nearest.x, nearest.y, new_x, new_y);
            int best_parent_idx = nearest_idx;
            
            // Find nearby nodes
            vector<int> nearby_indices;
            for (int i = 0; i < tree.size(); i++) {
                if (distance_xy(tree[i].x, tree[i].y, new_x, new_y) <= REWIRE_RADIUS) {
                    nearby_indices.push_back(i);
                }
            }
            
            // Find the cheapest collision-free connection
            for (int idx : nearby_indices) {
                double possible_cost = tree[idx].cost + distance_xy(tree[idx].x, tree[idx].y, new_x, new_y);
                if (possible_cost < min_cost && !collides(tree[idx].x, tree[idx].y, new_x, new_y, obstacles)) {
                    min_cost = possible_cost;
                    best_parent_idx = idx;
                }
            }
            
            q_new.parent_idx = best_parent_idx;
            q_new.cost = min_cost;
            int q_new_idx = tree.size();
            tree.push_back(q_new);
            
            // MODIFICATION 2: Rewire the tree
            for (int idx : nearby_indices) {
                double cost_via_new = q_new.cost + distance_xy(q_new.x, q_new.y, tree[idx].x, tree[idx].y);
                if (cost_via_new < tree[idx].cost && !collides(q_new.x, q_new.y, tree[idx].x, tree[idx].y, obstacles)) {
                    tree[idx].parent_idx = q_new_idx;
                    tree[idx].cost = cost_via_new;
                }
            }
            
            // Note: In RRT*, we usually let it run to MAX_ITER to fully optimize, 
            // but for comparison speed, we'll break when it hits the threshold.
            if (distance_xy(new_x, new_y, goal.x, goal.y) < GOAL_THRESHOLD) {
                vector<RRTNode> path;
                int curr = tree.size() - 1;
                while (curr != -1) {
                    path.push_back(tree[curr]);
                    curr = tree[curr].parent_idx;
                }
                reverse(path.begin(), path.end());
                return path;
            }
        }
    }
    return {};
}

// --- Main ---
int main() {
    srand(time(0));
    
    RRTNode start = {5, 5, -1, 0.0};
    RRTNode goal = {95, 95, -1, 0.0};
    
    vector<Rect> obstacles = {
        {20, 20, 20, 40},
        {60, 10, 15, 50},
        {30, 70, 40, 15}
    };
    
    cout << "--- PATHFINDING COMPARISON ---" << endl;
    
    // 1. Standard RRT
    vector<RRTNode> path_rrt = rrt(start, goal, obstacles);
    if (!path_rrt.empty()) {
        cout << "[RRT Standard]" << endl;
        cout << "Path Length: " << path_rrt.size() << " nodes" << endl;
        cout << "Total Cost:  " << calculate_path_cost(path_rrt) << endl << endl;
    }
    
    // 2. RRT* 
    vector<RRTNode> path_rrt_star = rrt_star(start, goal, obstacles);
    if (!path_rrt_star.empty()) {
        cout << "[RRT* (Star)]" << endl;
        cout << "Path Length: " << path_rrt_star.size() << " nodes" << endl;
        cout << "Total Cost:  " << calculate_path_cost(path_rrt_star) << endl << endl;
        
        // 3. RRT* + Smoothing
        smooth_path(path_rrt_star, obstacles);
        cout << "[RRT* + Smoothing]" << endl;
        cout << "Path Length: " << path_rrt_star.size() << " nodes" << endl;
        cout << "Total Cost:  " << calculate_path_cost(path_rrt_star) << endl << endl;
    } else {
        cout << "Path not found." << endl;
    }
    
    return 0;
}