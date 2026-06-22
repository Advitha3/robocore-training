# include <iostream>
# include <iomanip>
#include <random>

class KalmanFilter1D{
public:
    double x;//state estimate(altitude)
    double P;// Estimate uncertainty (covariance)
    double Q;// Process noise (trust in physiscs model)
    double R;//Measurement noise (trust in the sensor)
    double K;//Kalman gain

//constructor to initialt the filter parameters 

KalmanFilter1D(double initial_x, double initial_P, double process_noise, double measurement_noise){
    x = initial_x;
    P = initial_P;
    Q = process_noise;
    R = measurement_noise;
    K = 0.0;
}


//Phase 1: Predict (The blind guess)

void predict(double velocity, double dt){
    //Physics model : new position = old position + velocity * time
    x = x + (velocity * dt);
    // Uncertainty grows because our physics model isn't perfect
    P = P + Q;
}

// Phase 2 : Update(The reality Check)

void update(double measurement){
    //calculate kalman gain: how much do we trust the sensor vs, our prediction?
    K = P/ (P + R);

    //Correct the estimate by pulling it toward the measurement 
    x = x + K * (measurement -x);

    // Uncertainty shrinks because we just incorporated new data
    P = (1.0 -K) * P;
}
};

int main(){
    //---Setup the simulation ----
    double dt = 1.0; // 1 sec per cycle
    double velocity = 0.5; // drone climbs at 0.5 m/s
    double true_altitude = 0.0; //drone starts on the gnd

    //setup Random noise Generator (Mean = 0, stdDev = 2.0 meters)
    std::default_random_engine gen(42);
    std::normal_distribution<double> noise(0.0, 2.0);

    // Initialize Kalman Filter
    // Initial guess: 0m altitude. Initial uncertainty: 1.0. 
    // Q (Process noise): 0.1 (we trust our velocity model fairly well)
    // R (Measurement noise): 4.0 (Variance of std_dev 2.0 is 2^2 = 4)
    KalmanFilter1D kf(0.0, 1.0, 0.1, 4.0);

    std::cout << "--- 1D Kalman Filter Simulation: Drone Altitude ---\n\n";
    // ---Run 10 cycles ---
    for (int t = 1; t <=10; ++t){
        //1. simulation real world moving forward 
        true_altitude += (velocity * dt);

        //2. Generate a noisy sensor measurement 
        double measurement = true_altitude + noise(gen);

        // 3. Kalman Filter Step 1: Predict where we are based on velocity
        kf.predict(velocity, dt);
        
        // 4. Kalman Filter Step 2: Update based on the noisy measurement
        kf.update(measurement);

        std::cout << "[t=" << std::setw(2) << t << "] "
                  << std::fixed << std::setprecision(2)
                  << "True: " << std::setw(5) << true_altitude 
                  << " | Measured: " << std::setw(5) << measurement 
                  << " | Estimated: " << std::setw(5) << kf.x 
                  << " | P: " << std::setw(4) << kf.P 
                  << " | K: " << std::setw(4) << kf.K << "\n";

        int true_bar    = (int)(true_altitude * 4);
        int est_bar     = (int)(kf.x * 4);
        std::cout << "  True:      " << std::string(true_bar, '=') << "|\n";
        std::cout << "  Estimated: " << std::string(std::max(0,est_bar), '-') << "|\n\n";
    }

    return 0;
}

