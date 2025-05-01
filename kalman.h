class KalmanFilter {

public:

    /**
    * Create a Kalman filter with the specified matrices.
    *   A - System dynamics matrix
    *   C - Output matrix
    *   Q - Process noise covariance
    *   RPedometer - Pedometer measurement noise covariance
    *   HPedometer - Pedometer measurement matrix
    *   RGPS - GPS measurement noise covariance
    *   HGPS - GPS measurement matrix
    *   HLidar - Lidar measurement matrix
    *   RLidar - Lidar measurement noise covariance
    *   P - Estimate error covariance
    */
    KalmanFilter(
        double dt,
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& RPedometer,
        const Eigen::MatrixXd& HPedometer,
        const Eigen::MatrixXd& RGPS,
        const Eigen::MatrixXd& HGPS,
        const Eigen::MatrixXd& HLidar,
        const Eigen::MatrixXd& RLidar,
        const Eigen::MatrixXd& P
    );

    /**
    * Create a blank estimator.
    */
    KalmanFilter();

 


    void init(double t0, const Eigen::VectorXd& x0);

    void update(const Eigen::Vector2d& gpsMeasurement, const Eigen::Vector2d& lidarMeasurement,
        const Eigen::Vector2d& pedometerMeasurement);


    void update(const Eigen::Vector2d& gpsMeasurement, const Eigen::Vector2d& lidarMeasurement,
        const Eigen::Vector2d& pedometerMeasurement, double dt);

    void Sensor(const Eigen::VectorXd& y, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H);


    /**
    * Return the current state and time.
    */
    Eigen::VectorXd state() { return x_hat; };
    double time() { return t; };

private:

    // Matrices for computation
    Eigen::MatrixXd A, C, Q, RPedometer, HPedometer, RGPS, HGPS, HLidar, RLidar, P, K, P0;

    // System dimensions
    int m, n;

    // Initial and current time
    double t0, t;

    // Discrete time step
    double dt;

    // Is the filter initialized?
    bool initialized;

    // n-size identity
    Eigen::MatrixXd I;

    // Estimated states
    Eigen::VectorXd x_hat, x_hat_new;
};
