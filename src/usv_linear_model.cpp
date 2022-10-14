#include <usv_linear_model.hpp>

LinearModel::LinearModel(/* args */) {
  phi = Eigen::MatrixXd::Identity(STATE_COMPONENTS_, STATE_COMPONENTS_);
  q = Eigen::MatrixXd::Zero(STATE_COMPONENTS_, STATE_COMPONENTS_);
  q_dash = Eigen::MatrixXd::Zero(STATE_COMPONENTS_, STATE_COMPONENTS_);
  p_k_dash = Eigen::MatrixXd::Identity(STATE_COMPONENTS_, STATE_COMPONENTS_);
  l_k = Eigen::MatrixXd::Zero(STATE_COMPONENTS_, OUTPUT_COMPONENTS_);
  c_t = Eigen::MatrixXd::Zero(OUTPUT_COMPONENTS_, STATE_COMPONENTS_);
  x_t = Eigen::MatrixXd::Zero(STATE_COMPONENTS_, 1);
  x_predicted = x_t;
  w_k = Eigen::MatrixXd::Zero(OUTPUT_COMPONENTS_, 1);
  w_k_hat = w_k;
  p_k = Eigen::MatrixXd::Zero(STATE_COMPONENTS_, STATE_COMPONENTS_);
  r = Eigen::MatrixXd::Zero(OUTPUT_COMPONENTS_, OUTPUT_COMPONENTS_);
  c_t(0, 0) = 1.0;
  c_t(1, 1) = 1.0;
  c_t(2, 2) = 1.0;
  c_t(3, 3) = 1.0;
}

LinearModel::~LinearModel() {}



double LinearModel::getYaw(const geometry_msgs::PoseStamped &msg) {
  tf2::Quaternion quat(msg.pose.orientation.x, msg.pose.orientation.y,
                       msg.pose.orientation.z, msg.pose.orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  return yaw;
}

void LinearModel::initialiseModel(ros::NodeHandle &nh) {
	double r_temp = 0.0;
	double q_temp = 0.0;
	if (!nh.getParam("linear_model/r", r_temp)) {
		ROS_WARN("[LinearModel] : Couldn't load r");
	}
	if (!nh.getParam("linear_model/q", q_temp)) {
		ROS_WARN("[LinearModel] : Couldn't load q");
  }

  r(0, 0) = r_temp;
  r(1, 1) = r_temp;
  r(2, 2) = r_temp;
  r(3, 3) = r_temp;
  for (size_t i = 0; i < STATE_COMPONENTS_; i++) {
    q(i, i) = q_temp;
  }

  last_update_time_ = ros::Time::now().toSec();
}

void LinearModel::updateModel(const geometry_msgs::PoseStamped &msg) {
  double measurement_delta = ros::Time::now().toSec() - last_update_time_;
  last_update_time_ = ros::Time::now().toSec();
  w_k(0) = msg.pose.position.x;
  w_k(1) = msg.pose.position.y;
  w_k(2) = msg.pose.position.z;
  w_k(3) = getYaw(msg);
  phi(0, 4) = measurement_delta;
  phi(1, 5) = measurement_delta;
  phi(2, 6) = measurement_delta;
  phi(3, 7) = measurement_delta;
  q_dash = 0.5 * ((phi * q * phi.transpose()) + q) * measurement_delta;
  p_k_dash = ((phi * p_k_dash * phi.transpose()) + q_dash).eval();
  l_k = p_k_dash * c_t.transpose() *
        (c_t * p_k_dash * c_t.transpose() + r).inverse();
  x_predicted = phi * x_t;
  w_k_hat = c_t * x_predicted;
  x_predicted = (x_predicted + (l_k * (w_k - w_k_hat))).eval();
  w_k_hat = c_t * x_predicted;
  p_k = (Eigen::MatrixXd::Identity(STATE_COMPONENTS_, STATE_COMPONENTS_) -
         (l_k * c_t)) *
        p_k_dash;
  x_t = x_predicted;
  p_k_dash = p_k;
}

void LinearModel::getPrediction(geometry_msgs::Pose &msg, double time_elapsed) {
  if (time_elapsed == 0.0) {
    msg.position.x = x_t(0);
    msg.position.y = x_t(1);
    msg.position.z = x_t(2);
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0.0, 0.0, x_t(3));
    myQuaternion.normalize();
    msg.orientation.x = myQuaternion.getX();
    msg.orientation.y = myQuaternion.getY();
    msg.orientation.z = myQuaternion.getZ();
    msg.orientation.w = myQuaternion.getW();
    return;
  }
  double measurement_delta =
      ros::Time::now().toSec() - last_update_time_ + time_elapsed;

  x_predicted = x_t;
  phi(0, 4) = measurement_delta;
  phi(1, 5) = measurement_delta;
  phi(2, 6) = measurement_delta;
  phi(3, 7) = measurement_delta;
  x_predicted = phi * x_predicted;
  msg.position.x = x_predicted(0);
  msg.position.y = x_predicted(1);
  msg.position.z = x_predicted(2);
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0.0, 0.0, x_predicted(3));
  myQuaternion.normalize();
  msg.orientation.x = myQuaternion.getX();
  msg.orientation.y = myQuaternion.getY();
  msg.orientation.z = myQuaternion.getZ();
  msg.orientation.w = myQuaternion.getW();
}

double LinearModel::getCovarianceOfVxy() {
  return p_k_dash(4, 4) + p_k_dash(5, 5);
}
