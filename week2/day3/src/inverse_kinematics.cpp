#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>

// Pi constant for conversions
const double PI = 3.14159265358979323846;

// Helper function to convert radians to degrees
double RadToDeg(double radians) {
    return radians * (180.0 / PI);
}

// 2D Vector structure
struct Vec2 {
    double x;
    double y;
};

// 4x4 Matrix structure for Transformations (simulating Day 1 logic)
struct Mat4 {
    double m[4][4] = {0};

    // Constructor creates an Identity matrix by default
    Mat4() {
        m[0][0] = 1; m[1][1] = 1; m[2][2] = 1; m[3][3] = 1;
    }

    // Matrix Multiplication
    Mat4 operator*(const Mat4& other) const {
        Mat4 result;
        // Zero out the identity diagonal first
        result.m[0][0]=0; result.m[1][1]=0; result.m[2][2]=0; result.m[3][3]=0; 
        
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                for(int k = 0; k < 4; k++) {
                    result.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return result;
    }

    // Multiply Matrix by a 2D point (padding with Z=0, W=1)
    Vec2 multiplyPoint(const Vec2& p) const {
        double newX = m[0][0]*p.x + m[0][1]*p.y + m[0][2]*0.0 + m[0][3]*1.0;
        double newY = m[1][0]*p.x + m[1][1]*p.y + m[1][2]*0.0 + m[1][3]*1.0;
        return {newX, newY};
    }

    static Mat4 Translation(double x, double y) {
        Mat4 t;
        t.m[0][3] = x;
        t.m[1][3] = y;
        return t;
    }

    static Mat4 RotationZ(double radians) {
        Mat4 r;
        r.m[0][0] = cos(radians);  r.m[0][1] = -sin(radians);
        r.m[1][0] = sin(radians);  r.m[1][1] = cos(radians);
        return r;
    }
};

// Structure to hold our Inverse Kinematic results
struct IKSolution {
    bool reachable;
    double theta1_up;     // Radians
    double theta2_up;     // Radians
    double theta1_down;   // Radians
    double theta2_down;   // Radians
};

// Main Robot Arm structure
struct RobotArm {
    double L1;
    double L2;

    // --- FORWARD KINEMATICS ---
    Vec2 ForwardKinematics(double theta1, double theta2) {
        // Transformation from Base to Joint 1
        Mat4 T_base_j1 = Mat4::RotationZ(theta1) * Mat4::Translation(L1, 0.0);
        
        // Transformation from Joint 1 to End Effector
        Mat4 T_j1_ee = Mat4::RotationZ(theta2) * Mat4::Translation(L2, 0.0);
        
        // Combined Transformation
        Mat4 T_final = T_base_j1 * T_j1_ee;
        
        // The origin point (0,0) of the end effector, transformed to base coordinates
        Vec2 origin = {0.0, 0.0};
        return T_final.multiplyPoint(origin);
    }

    // --- INVERSE KINEMATICS ---
    IKSolution InverseKinematics(double x, double y) {
        IKSolution sol;
        
        // 1. Calculate D (Cosine of theta2)
        double D = (x*x + y*y - L1*L1 - L2*L2) / (2.0 * L1 * L2);

        // 2. Check if the target is reachable
        if (D > 1.0 || D < -1.0) {
            sol.reachable = false;
            return sol;
        }
        sol.reachable = true;

        // 3. Solution 1: Elbow UP (Positive square root)
        sol.theta2_up = atan2(sqrt(1.0 - D*D), D);
        double k1_up = L1 + L2 * cos(sol.theta2_up);
        double k2_up = L2 * sin(sol.theta2_up);
        sol.theta1_up = atan2(y, x) - atan2(k2_up, k1_up);

        // 4. Solution 2: Elbow DOWN (Negative square root)
        sol.theta2_down = atan2(-sqrt(1.0 - D*D), D);
        double k1_down = L1 + L2 * cos(sol.theta2_down);
        double k2_down = L2 * sin(sol.theta2_down);
        sol.theta1_down = atan2(y, x) - atan2(k2_down, k1_down);

        return sol;
    }
};

int main() {
    RobotArm arm = {1.0, 1.0};

    // List of targets to test
    std::vector<Vec2> targets = {
        {1.0, 1.0},
        {1.5, 0.5},
        {2.0, 0.0}, // The true fully-extended special case (L1 + L2 = 2.0)
        {3.0, 0.0}, // Unreachable 
        {5.0, 0.0}  // Unreachable
    };

    // Output formatting
    std::cout << std::fixed << std::setprecision(1);

    for (const auto& target : targets) {
        std::cout << "Target: [" << target.x << ", " << target.y << "]\n";

        IKSolution ik = arm.InverseKinematics(target.x, target.y);

        if (!ik.reachable) {
            std::cout << "  Status: Target is UNREACHABLE! (Out of range)\n\n";
            continue;
        }

        // Print Elbow UP Data
        std::cout << "  Elbow UP:   θ1=" << RadToDeg(ik.theta1_up) << "°  "
                  << "θ2=" << RadToDeg(ik.theta2_up) << "°\n";
        
        // Print Elbow DOWN Data
        std::cout << "  Elbow DOWN: θ1=" << RadToDeg(ik.theta1_down) << "°  "
                  << "θ2=" << RadToDeg(ik.theta2_down) << "°\n";

        // Verify with Forward Kinematics (using Elbow Up as test)
        Vec2 fk_verify = arm.ForwardKinematics(ik.theta1_up, ik.theta2_up);
        std::cout << "  Verify FK:  [" << fk_verify.x << ", " << fk_verify.y << "]\n\n";
    }

    return 0;
}