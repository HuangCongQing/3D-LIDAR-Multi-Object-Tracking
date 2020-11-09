#ifndef MY_PCL_TUTORIAL_UKF_H
#define MY_PCL_TUTORIAL_UKF_H
// #include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

//    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    MatrixXd x_merge_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    MatrixXd x_cv_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    MatrixXd x_ctrv_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    MatrixXd x_rm_;

//    ///* state covariance matrix
    MatrixXd P_merge_;

    ///* state covariance matrix
    MatrixXd P_cv_;

    ///* state covariance matrix
    MatrixXd P_ctrv_;

    ///* state covariance matrix
    MatrixXd P_rm_;

//    ///* predicted sigma points matrix
//    MatrixXd Xsig_pred_;

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_cv_;

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_ctrv_;

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_rm_;

    ///* time when the state is true, in us
    long long time_us_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_cv_;
    double std_a_ctrv_;
    double std_a_rm_;
    ///* Process noise standard deviation yaw acceleration in rad/s^2
//    double std_yawdd_;
    // CTRV
    double std_ctrv_yawdd_;
    // CV
    double std_cv_yawdd_;

    double std_rm_yawdd_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;

    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_ ;

    ///* Weights of sigma points
    VectorXd weights_;

    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///* Sigma point spreading parameter
    double lambda_;

    ///* Augmented sigma point spreading parameter
    double lambda_aug_;

    ///* the current NIS for radar
    double NIS_radar_;

    ///* the current NIS for laser
    double NIS_laser_;

    int count_;
    int count_empty_;

    double modeMatchProbCV2CV_;
    double modeMatchProbCTRV2CV_;
    double modeMatchProbRM2CV_ ;

    double modeMatchProbCV2CTRV_;
    double modeMatchProbCTRV2CTRV_ ;
    double modeMatchProbRM2CTRV_ ;

    double modeMatchProbCV2RM_ ;
    double modeMatchProbCTRV2RM_ ;
    double modeMatchProbRM2RM_ ;

    double modeMatchProbCV_;

    double modeMatchProbCTRV_;

    double modeMatchProbRM_;

    double modeProbCV_;
    double modeProbCTRV_;
    double modeProbRM_;

    std::vector<double> ini_u_;

    std::vector<double> p1_;

    std::vector<double> p2_;

    std::vector<double> p3_;

    VectorXd zPredCVr_;
    VectorXd zPredCTRVr_;
    VectorXd zPredRMr_;

    VectorXd zPredCVl_;
    VectorXd zPredCTRVl_;
    VectorXd zPredRMl_;

    MatrixXd lS_cv_;
    MatrixXd lS_ctrv_;
    MatrixXd lS_rm_;
    MatrixXd rS_cv_;
    MatrixXd rS_ctrv_;
    MatrixXd rS_rm_;

    MatrixXd K_cv_;
    MatrixXd K_ctrv_;
    MatrixXd K_rm_;

    // Output filestreams for radar and laser NIS
//    std::ofstream NISvals_laser_cv_;
//    std::ofstream NISvals_laser_ctrv_;
//    std::ofstream NISvals_laser_rm_;

    double gammaG_;
    double pD_;
    double pG_;

    int lifetime_;
    std::vector<double> velo_history_;
    bool isStatic_;

    // bounding box params
    bool isVisBB_;
    // todo: need initialization?
    pcl::PointCloud<pcl::PointXYZ> BBox_;
    pcl::PointCloud<pcl::PointXYZ> bestBBox_;
    double bestYaw_;
    double bb_yaw_;
    double bb_area_;
    std::vector<double> bb_yaw_history_;
    std::vector<double> bb_vel_history_;
    std::vector<double> bb_area_history_;

    // for env classification
    VectorXd initMeas_;
    double distFromInit_;

    
    std::vector<VectorXd> local2local_;
    std::vector<double> local2localYawVec_;

    double x_merge_yaw_;

    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    void UpdateYawWithHighProb();

    void Initialize(VectorXd z, double timestamp);

    double CalculateGauss(VectorXd z, int sensorInd, int modelInd);

    void UpdateModeProb(std::vector<double> lambdaVec);

    void MergeEstimationAndCovariance();

    void MixingProbability();

    void Interaction();

    void MeasurementValidation(VectorXd z, std::vector<VectorXd>& meas);

    void PDAupdate(std::vector<VectorXd> z, int modelInd);

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessIMMUKF(double dt);

    void PostProcessIMMUKF(std::vector<double> lambdaVec);


    void Ctrv(double p_x, double p_y, double v, double yaw, double yawd, double nu_a, double nu_yawdd, double delta_t, std::vector<double>&state);

    void Cv(double p_x, double p_y, double v, double yaw, double yawd, double nu_a, double nu_yawdd, double delta_t, std::vector<double>&state);

    void randomMotion(double p_x, double p_y, double v, double yaw, double yawd, double nu_a, double nu_yawdd, double delta_t, std::vector<double>&state);

    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t, int modelInd);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(int modelInd);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    // void UpdateRadar(MeasurementPackage meas_package, int modelInd);
};

#endif /* UKF_H */
