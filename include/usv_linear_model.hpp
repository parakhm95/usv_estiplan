#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>

class LinearModel {
private:
  double last_update_time_;
  double OUTPUT_COMPONENTS_ = 4;
  double STATE_COMPONENTS_ = 8;

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
  geometry_msgs::Pose msg_prediction_;
  /* data */
public:
  LinearModel(/* args */);
  ~LinearModel();
  void updateModel(const geometry_msgs::PoseStamped &msg);
  void initialiseModel(ros::NodeHandle &nh);
  double getYaw(const geometry_msgs::PoseStamped &msg);
  void getPrediction(geometry_msgs::Pose &msg, double time_elapsed);
  Eigen::MatrixXd getCovarianceOfPrediction(double elapsed_time);
};
