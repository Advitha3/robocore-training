#include <iostream>
#include <iomanip>
#include <random>

// 1. Lightweight 2D Vector & Matrix Structs
struct Vec2 {
    double z, v; // Altitude and Velocity
};

struct Mat2x2 {
    double data[2][2] = {{0.0}};

    Mat2x2 operator*(const Mat2x2& other) const {
        Mat2x2 res;
        for (int i=0; i<2; ++i)
            for (int j=0; j<2; ++j)
                for (int k=0; k<2; ++k)
                    res.data[i][j] += data[i][k] * other.data[k][j];
        return res;
    }

    Mat2x2 operator+(const Mat2x2& other) const {
        Mat2x2 res;
        for (int i=0; i<2; ++i)
            for (int j=0; j<2; ++j)
                res.data[i][j] = data[i][j] + other.data[i][j];
        return res;
    }

    Mat2x2 transpose() const {
        Mat2x2 res;
        res.data[0][0] = data[0][0]; res.data[0][1] = data[1][0];
        res.data[1][0] = data[0][1]; res.data[1][1] = data[1][1];
        return res;
    }
};

// 2. The Extended Kalman Filter Class
class EKF {
public:
    Vec2 x;     // State: [altitude, velocity]
    Mat2x2 P;   // Covariance (Uncertainty)
    Mat2x2 Q;   // Process Noise
    double R;   // Measurement Noise

    EKF(double z0, double v0) {
        x.z = z0; x.v = v0;
        
        // Initial uncertainty (we are fairly unsure)
        P.data[0][0] = 1.0; P.data[1][1] = 1.0; 
        
        // Process noise (Q): We trust our drag physics, but not perfectly
        Q.data[0][0] = 0.1; Q.data[1][1] = 0.1; 
        
        // Measurement noise (R): Barometer variance
        R = 4.0; 
    }

    void predict(double dt) {
        // 1. Non-linear State Update
        double z_new = x.z + x.v * dt;
        double v_new = x.v - 0.1 * (x.v * x.v) * dt; // Drag formula!
        
        // 2. Compute dynamic Jacobian (F)
        Mat2x2 F;
        F.data[0][0] = 1.0; 
        F.data[0][1] = dt;
        F.data[1][0] = 0.0; 
        F.data[1][1] = 1.0 - 0.2 * x.v * dt; // Changing slope!

        // 3. Apply updates
        x.z = z_new;
        x.v = v_new;
        P = (F * P * F.transpose()) + Q;
    }

    void update(double measurement) {
        // Because H = [1, 0], matrix math simplifies neatly to algebraic steps
        // Innovation (Residual)
        double y = measurement - x.z;
        
        // S = H*P*H^T + R (simplifies to just P00 + R)
        double S = P.data[0][0] + R;
        
        // Kalman Gain K = P*H^T * S^-1
        double K_z = P.data[0][0] / S; // Gain for altitude
        double K_v = P.data[1][0] / S; // Gain for velocity (Hidden state!)

        // State Update
        x.z = x.z + K_z * y;
        x.v = x.v + K_v * y;

        // Covariance Update P = (I - K*H)*P
        Mat2x2 P_new;
        P_new.data[0][0] = (1.0 - K_z) * P.data[0][0];
        P_new.data[0][1] = (1.0 - K_z) * P.data[0][1];
        P_new.data[1][0] = P.data[1][0] - K_v * P.data[0][0];
        P_new.data[1][1] = P.data[1][1] - K_v * P.data[0][1];
        P = P_new;
    }
};

// 3. Simulation Environment
int main() {
    double dt = 1.0;
    
    // The "Real World" variables
    double true_z = 0.0;
    double true_v = 8.0; // Launch upwards at 8 m/s
    
    std::default_random_engine gen(42); 
    std::normal_distribution<double> noise(0.0, 2.0);

    // Initialize EKF
    EKF ekf(0.0, 8.0);

    std::cout << "--- 2D Extended Kalman Filter: Non-Linear Drag ---\n\n";

    for (int t = 1; t <= 10; ++t) {
        // 1. Real World Physics moves forward
        true_z = true_z + true_v * dt;
        true_v = true_v - 0.1 * (true_v * true_v) * dt; // Air drag slows it down!
        
        // 2. Take noisy barometer reading
        double measurement = true_z + noise(gen);

        // 3. Run EKF
        ekf.predict(dt);
        ekf.update(measurement);

        std::cout << "[t=" << std::setw(2) << t << "] " << std::fixed << std::setprecision(2)
                  << "True Alt: " << std::setw(5) << true_z 
                  << " | Est Alt: " << std::setw(5) << ekf.x.z 
                  << " || True Vel: " << std::setw(5) << true_v
                  << " | Est Vel: " << std::setw(5) << ekf.x.v << "\n";
    }

    return 0;
}