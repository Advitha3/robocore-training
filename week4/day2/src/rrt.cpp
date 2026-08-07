#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

using namespace std;

// --- Structures ---
struct RRTNode {
    double x, y;
    int parent_idx; // -1 for root
};

struct Rect {
    double x, y, w, h;
};

// --- Constants ---
const double STEP_SIZE = 5.0;
const int MAX_ITER = 5000;
const double GOAL_THRESHOLD = 5.0;

// --- Helper Functions ---
double distance(const RRTNode& a, const RRTNode& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

int nearest_node(vector<RRTNode>& tree, double x, double y) {
    int best_idx = -1;
    double min_dist = 1e9; // Start with a very large distance
    RRTNode target{x, y, -1};
    
    for (size_t i = 0; i < tree.size(); ++i) {
        double d = distance(tree[i], target);
        if (d < min_dist) {
            min_dist = d;
            best_idx = i;
        }
    }
    return best_idx;
}

bool collides(double x1, double y1, double x2, double y2, vector<Rect>& obstacles) {
    // Sample 10 points along the line segment
    for (int i = 0; i <= 10; ++i) {
        double t = i / 10.0;
        double px = x1 + t * (x2 - x1);
        double py = y1 + t * (y2 - y1);
        
        // Check if point (px, py) is inside any of the rectangular obstacles
        for (const auto& obs : obstacles) {
            if (px >= obs.x && px <= obs.x + obs.w &&
                py >= obs.y && py <= obs.y + obs.h) {
                return true; // Collision detected
            }
        }
    }
    return false; // Path is clear
}

pair<double, double> steer(RRTNode& from, double to_x, double to_y) {
    double theta = atan2(to_y - from.y, to_x - from.x);
    double new_x = from.x + STEP_SIZE * cos(theta);
    double new_y = from.y + STEP_SIZE * sin(theta);
    return {new_x, new_y};
}

// --- RRT Algorithm ---
vector<RRTNode> rrt(RRTNode start, RRTNode goal, vector<Rect>& obstacles, int& iter_count) {
    vector<RRTNode> tree;
    tree.push_back(start);
    
    // Setup random number generation for [0, 100]
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> dist(0.0, 100.0);
    
    for (iter_count = 1; iter_count <= MAX_ITER; ++iter_count) {
        // 1. Sample random point
        double rand_x = dist(gen);
        double rand_y = dist(gen);
        
        // 2. Find nearest node in the tree
        int nearest_idx = nearest_node(tree, rand_x, rand_y);
        RRTNode nearest = tree[nearest_idx];
        
        // 3. Steer towards the random point
        pair<double, double> new_point = steer(nearest, rand_x, rand_y);
        
        // 4. Check for collision
        if (!collides(nearest.x, nearest.y, new_point.first, new_point.second, obstacles)) {
            // Add new node to the tree
            RRTNode new_node{new_point.first, new_point.second, nearest_idx};
            tree.push_back(new_node);
            
            // 5. Check if we reached the goal
            if (distance(new_node, goal) <= GOAL_THRESHOLD) {
                // Add the exact goal node to finish the path cleanly
                tree.push_back({goal.x, goal.y, (int)tree.size() - 1});
                return tree;
            }
        }
    }
    
    return tree; // Max iterations reached without finding a path
}

int main() {
    // Initialize Start and Goal
    RRTNode start{5.0, 5.0, -1};
    RRTNode goal{95.0, 95.0, -1};
    
    // Initialize Obstacles
    vector<Rect> obstacles = {
        {20.0, 20.0, 20.0, 40.0},
        {60.0, 10.0, 15.0, 50.0},
        {30.0, 70.0, 40.0, 15.0}
    };
    
    int iters_taken = 0;
    
    cout << "Running RRT Planner..." << endl;
    vector<RRTNode> tree = rrt(start, goal, obstacles, iters_taken);
    
    // Check if goal was reached
    if (distance(tree.back(), goal) > GOAL_THRESHOLD && iters_taken > MAX_ITER) {
        cout << "Failed to find a path within max iterations." << endl;
        return 1;
    }
    
    // Reconstruct Path
    vector<RRTNode> path;
    int current_idx = tree.size() - 1; // Start from the last node added (goal)
    
    while (current_idx != -1) {
        path.push_back(tree[current_idx]);
        current_idx = tree[current_idx].parent_idx;
    }
    
    // Reverse to get path from Start -> Goal
    reverse(path.begin(), path.end());
    
    // Print Results
    cout << "\nPath found in " << iters_taken << " iterations!" << endl;
    cout << "Path length: " << path.size() << " nodes" << endl;
    
    cout << "Path coordinates:" << endl;
    for (size_t i = 0; i < path.size(); ++i) {
        cout << "(" << path[i].x << ", " << path[i].y << ")";
        if (i < path.size() - 1) cout << " -> ";
        if ((i + 1) % 4 == 0) cout << endl; // Break line for readability
    }
    cout << endl;

    return 0;
}