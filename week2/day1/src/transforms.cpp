#include <iostream>
#include <cmath>
#include <iomanip>

// 1. Vector Struct
struct Vec3 {
    double x, y, z;

    void print() const {
        double px = (std::abs(x) < 1e-10) ? 0.0 : x;
        double py = (std::abs(y) < 1e-10) ? 0.0 : y;
        double pz = (std::abs(z) < 1e-10) ? 0.0 : z;
        std::cout << "[" << px << ", " << py << ", " << pz << "]\n";
    }
};

// 2. 3x3 Matrix Struct
struct Matrix3x3 {
    double data[3][3] = {{0.0}};

    Matrix3x3 operator*(const Matrix3x3& other) const {
        Matrix3x3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                for (int k = 0; k < 3; ++k) {
                    result.data[i][j] += data[i][k] * other.data[k][j];
                }
            }
        }
        return result;
    }

    Vec3 operator*(const Vec3& v) const {
        Vec3 result;
        result.x = data[0][0]*v.x + data[0][1]*v.y + data[0][2]*v.z;
        result.y = data[1][0]*v.x + data[1][1]*v.y + data[1][2]*v.z;
        result.z = data[2][0]*v.x + data[2][1]*v.y + data[2][2]*v.z;
        return result;
    }

    void print() const {
        for (int i = 0; i < 3; ++i) {
            std::cout << "[";
            for (int j = 0; j < 3; ++j) {
                double val = (std::abs(data[i][j]) < 1e-10) ? 0.0 : data[i][j];
                std::cout << std::setw(8) << std::fixed << std::setprecision(4) << val << " ";
            }
            std::cout << "]\n";
        }
    }
};

// 3. Rotation Matrix Generators
Matrix3x3 rot_z(double angle_rad) {
    Matrix3x3 m;
    m.data[0][0] = std::cos(angle_rad);  m.data[0][1] = -std::sin(angle_rad);
    m.data[1][0] = std::sin(angle_rad);  m.data[1][1] = std::cos(angle_rad);
    m.data[2][2] = 1.0;
    return m;
}

// 4. Homogeneous 4x4 Transformation Matrix Struct
struct Transform4x4 {
    double data[4][4] = {{0.0}};

    // Default constructor
    Transform4x4() {}

    // Constructor: Embeds the 3x3 Rotation and 3x1 Translation
    Transform4x4(const Matrix3x3& R, const Vec3& t) {
        // Copy Rotation Matrix into Top-Left 3x3
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                data[i][j] = R.data[i][j];
            }
        }
        // Copy Translation Vector into Top-Right 3x1
        data[0][3] = t.x;
        data[1][3] = t.y;
        data[2][3] = t.z;
        
        // Set Bottom Row to [0, 0, 0, 1]
        data[3][0] = 0.0;
        data[3][1] = 0.0;
        data[3][2] = 0.0;
        data[3][3] = 1.0;
    }

    // Matrix * Matrix (Standard 4x4 multiplication)
    Transform4x4 operator*(const Transform4x4& other) const {
        Transform4x4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                for (int k = 0; k < 4; ++k) {
                    result.data[i][j] += data[i][k] * other.data[k][j];
                }
            }
        }
        return result;
    }

    // Matrix * Vector (Automatically handles the w=1 homogeneous coordinate)
    Vec3 operator*(const Vec3& v) const {
        Vec3 result;
        // x, y, z are multiplied by the vector + the translation column * 1.0
        result.x = data[0][0]*v.x + data[0][1]*v.y + data[0][2]*v.z + data[0][3]*1.0;
        result.y = data[1][0]*v.x + data[1][1]*v.y + data[1][2]*v.z + data[1][3]*1.0;
        result.z = data[2][0]*v.x + data[2][1]*v.y + data[2][2]*v.z + data[2][3]*1.0;
        return result;
    }

    void print() const {
        for (int i = 0; i < 4; ++i) {
            std::cout << "[";
            for (int j = 0; j < 4; ++j) {
                double val = (std::abs(data[i][j]) < 1e-10) ? 0.0 : data[i][j];
                std::cout << std::setw(8) << std::fixed << std::setprecision(4) << val << " ";
                if (j == 2) std::cout << " | "; // Visual separator for the translation column
            }
            std::cout << "]\n";
            if (i == 2) {
                std::cout << "-------------------------------------------\n"; // Visual separator for the bottom row
            }
        }
    }
};

// 5. Main Forward Kinematics Simulation
int main() {
    // --- Joint 1 ---
    // Rotates 45 degrees around Z, link length is 1 unit along X
    double theta1 = 45.0 * (M_PI / 180.0);
    Vec3 t1 = {1.0, 0.0, 0.0};
    Transform4x4 T1(rot_z(theta1), t1);

    std::cout << "T1 (Joint 1 to Base):\n";
    T1.print();
    std::cout << "\n";

    // --- Joint 2 ---
    // Rotates 30 degrees around Z, link length is 1 unit along X
    double theta2 = 30.0 * (M_PI / 180.0);
    Vec3 t2 = {1.0, 0.0, 0.0};
    Transform4x4 T2(rot_z(theta2), t2);

    double theta3 = 45.0 * (M_PI / 180.0);
    Vec3 t3 = {0.5, 0.0, 0.0};
    Transform4x4 T3(rot_z(theta3), t3);

    std::cout << "T2 (Joint 2 to Joint 1):\n";
    T2.print();
    std::cout << "\n";

    // --- Forward Kinematics ---
    // T_final = T1 * T2
    Transform4x4 T_final = T1 * T2;
    std::cout << "T_final (End Effector to Base):\n";
    T_final.print();
    std::cout << "\n";

    // Apply the final transform to the end effector's local origin [0, 0, 0]
    Vec3 local_origin = {0.0, 0.0, 0.0};
    Vec3 world_position = T_final * local_origin;

    std::cout << "Final End Effector Position in World Space:\n";
    world_position.print();

    return 0;
}