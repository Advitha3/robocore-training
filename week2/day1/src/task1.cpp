#include <iostream>
#include <cmath>
#include <iomanip>

// 1. Vector Struct
// Defined first so Matrix3x3 knows it exists
struct Vec3 {
    double x, y, z;

    void print() const {
        // Cleaning up microscopic floating-point errors (e.g., 6.12323e-17) for clean output
        double px = (std::abs(x) < 1e-10) ? 0.0 : x;
        double py = (std::abs(y) < 1e-10) ? 0.0 : y;
        double pz = (std::abs(z) < 1e-10) ? 0.0 : z;
        std::cout << "[" << px << ", " << py << ", " << pz << "]\n";
    }
};

// 2. Matrix Struct
struct Matrix3x3 {
    // Initialize all elements to 0 by default
    double data[3][3] = {{0.0}}; 

    // Operator Overload: Matrix * Matrix
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

    // Operator Overload: Matrix * Vector
    Vec3 operator*(const Vec3& v) const {
        Vec3 result;
        result.x = data[0][0]*v.x + data[0][1]*v.y + data[0][2]*v.z;
        result.y = data[1][0]*v.x + data[1][1]*v.y + data[1][2]*v.z;
        result.z = data[2][0]*v.x + data[2][1]*v.y + data[2][2]*v.z;
        return result;
    }

    // Transpose method (swaps rows and columns)
    Matrix3x3 transpose() const {
        Matrix3x3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.data[i][j] = data[j][i];
            }
        }
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
Matrix3x3 rot_x(double angle_rad) {
    Matrix3x3 m;
    m.data[0][0] = 1.0;
    m.data[1][1] = std::cos(angle_rad);  m.data[1][2] = -std::sin(angle_rad);
    m.data[2][1] = std::sin(angle_rad);  m.data[2][2] = std::cos(angle_rad);
    return m;
}

Matrix3x3 rot_y(double angle_rad) {
    Matrix3x3 m;
    m.data[0][0] = std::cos(angle_rad);   m.data[0][2] = std::sin(angle_rad);
    m.data[1][1] = 1.0;
    m.data[2][0] = -std::sin(angle_rad);  m.data[2][2] = std::cos(angle_rad);
    return m;
}

Matrix3x3 rot_z(double angle_rad) {
    Matrix3x3 m;
    m.data[0][0] = std::cos(angle_rad);  m.data[0][1] = -std::sin(angle_rad);
    m.data[1][0] = std::sin(angle_rad);  m.data[1][1] = std::cos(angle_rad);
    m.data[2][2] = 1.0;
    return m;
}

// 4. Main Function to prove it works
int main() {
    // Create a unit vector pointing along X
    Vec3 v = {1.0, 0.0, 0.0};
    
    std::cout << "Original Vector v:\n";
    v.print();
    std::cout << "-----------------------\n";

    // 90 degrees in radians (Pi / 2)
    double angle = M_PI / 2.0; 
    
    // Generate the rotation matrix for Z
    Matrix3x3 R_z = rot_z(angle);
    
    std::cout << "Rotation Matrix R_z (90 degrees):\n";
    R_z.print();
    std::cout << "-----------------------\n";

    // Multiply the matrix by the vector
    Vec3 v_rotated = R_z * v;
    
    std::cout << "Rotated Vector v_rotated:\n";
    v_rotated.print();

    return 0;
}