#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <usv_estiplan/OdometryArray.h>

/* for loading dynamic parameters while the nodelet is running */
#include <dynamic_reconfigure/server.h>

/* this header file is created during compilation from python script
 * dynparam.cfg */
#include <usv_estiplan/dynparamConfig.h>

#include <Eigen/Dense>

class LinearInputModel {
private:
  double last_update_time_;
  double OUTPUT_COMPONENTS_ = 4;
  double STATE_COMPONENTS_ = 8;
  double drag_multiplier_ = 2.0;
  bool was_wrap_executed_ = false;
    bool first_start = true;

  Eigen::MatrixXd phi;
  Eigen::MatrixXd q;
  Eigen::MatrixXd q_dash;
  Eigen::MatrixXd p_k;
  Eigen::MatrixXd p_k_dash;
  Eigen::MatrixXd l_k;
  Eigen::MatrixXd c_t;
  Eigen::MatrixXd x_t;
  Eigen::MatrixXd x_predicted;
  Eigen::MatrixXd w_k;
  Eigen::MatrixXd r;
  Eigen::MatrixXd w_k_hat;
  Eigen::MatrixXd input_mat;
  geometry_msgs::Pose msg_prediction_;

  /* data */
public:
  LinearInputModel(/* args */);
  ~LinearInputModel();
  void updateModel(const geometry_msgs::PoseStamped &msg);
  void initialiseModel(ros::NodeHandle &nh);
  double getYaw(const geometry_msgs::PoseStamped &msg);
  void getPrediction(geometry_msgs::Pose &msg, double time_elapsed);
  Eigen::MatrixXd getCovarianceOfPrediction(double elapsed_time);
  Eigen::VectorXd calculateInput(const Eigen::VectorXd &current_state);
  void iterateModel();
  Eigen::VectorXd returnIteratedState(const Eigen::VectorXd &input_state,double timestep);
  void returnPredictions(double time_elapsed,
                         usv_estiplan::OdometryArray &msg_odom_array, double timestep);
  double wrapHeading(double heading, bool &was_wrap_executed);
  double solveHeading(const double &reported_yaw, const double &current_yaw);
  void callbackDynamicReconfigure([[maybe_unused]] usv_estiplan::dynparamConfig &config,
                                  [[maybe_unused]] uint32_t level);
  typedef dynamic_reconfigure::Server<usv_estiplan::dynparamConfig>
      ReconfigureServer;
  boost::recursive_mutex mutex_dynamic_reconfigure_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  usv_estiplan::dynparamConfig last_drs_config_;
};
