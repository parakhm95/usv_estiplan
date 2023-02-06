#include <usv_linear_input_model.hpp>

LinearInputModel::LinearInputModel(/* args */) {
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

LinearInputModel::~LinearInputModel() {}

double LinearInputModel::getYaw(const geometry_msgs::PoseStamped &msg) {
  tf2::Quaternion quat(msg.pose.orientation.x, msg.pose.orientation.y,
                       msg.pose.orientation.z, msg.pose.orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  return yaw;
}

void LinearInputModel::initialiseModel(ros::NodeHandle &nh) {
  if (!nh.getParam("linear_model/r_xy", r(0, 0))) {
    ROS_WARN("[LinearModel] : Couldn't load r_xy");
  }
  if (!nh.getParam("linear_model/r_xy", r(1, 1))) {
    ROS_WARN("[LinearModel] : Couldn't load r_xy");
  }
  if (!nh.getParam("linear_model/r_z", r(2, 2))) {
    ROS_WARN("[LinearModel] : Couldn't load r_z");
  }
  if (!nh.getParam("linear_model/r_heading", r(3, 3))) {
    ROS_WARN("[LinearModel] : Couldn't load r_heading");
  }
  if (!nh.getParam("linear_model/q_xy", q(0, 0))) {
    ROS_WARN("[LinearModel] : Couldn't load q_xy");
  }
  if (!nh.getParam("linear_model/q_xy", q(1, 1))) {
    ROS_WARN("[LinearModel] : Couldn't load q_xy");
  }
  if (!nh.getParam("linear_model/q_z", q(2, 2))) {
    ROS_WARN("[LinearModel] : Couldn't load q_z");
  }
  if (!nh.getParam("linear_model/q_heading", q(3, 3))) {
    ROS_WARN("[LinearModel] : Couldn't load q_heading");
  }
  if (!nh.getParam("linear_model/q_v_xy", q(4, 4))) {
    ROS_WARN("[LinearModel] : Couldn't load q_v_xy");
  }
  if (!nh.getParam("linear_model/q_v_xy", q(5, 5))) {
    ROS_WARN("[LinearModel] : Couldn't load q_v_xy");
  }
  if (!nh.getParam("linear_model/q_v_z", q(6, 6))) {
    ROS_WARN("[LinearModel] : Couldn't load q_v_z");
  }
  if (!nh.getParam("linear_model/q_heading_rate", q(7, 7))) {
    ROS_WARN("[LinearModel] : Couldn't load q_heading_rate");
  }
  if (!nh.getParam("linear_model/water_drag", drag_multiplier)) {
    ROS_WARN("[LinearModel] : Couldn't load water_drag");
  }

  last_update_time_ = ros::Time::now().toSec();
}

void LinearInputModel::updateModel(const geometry_msgs::PoseStamped &msg) {
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
  x_predicted = phi * x_t + measurement_delta * calculateInput(x_predicted);
  w_k_hat = c_t * x_predicted;
  x_predicted = (x_predicted + (l_k * (w_k - w_k_hat))).eval();
  w_k_hat = c_t * x_predicted;
  p_k = (Eigen::MatrixXd::Identity(STATE_COMPONENTS_, STATE_COMPONENTS_) -
         (l_k * c_t)) *
        p_k_dash;
  x_t = x_predicted;
  p_k_dash = p_k;
}

void LinearInputModel::getPrediction(geometry_msgs::Pose &msg,
                                     double time_elapsed) {
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
  x_predicted = phi * x_predicted + 0.01 * calculateInput(x_predicted);
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

Eigen::MatrixXd
LinearInputModel::getCovarianceOfPrediction(double elapsed_time) {

  double measurement_delta =
      ros::Time::now().toSec() - last_update_time_ + elapsed_time;
  phi(0, 4) = measurement_delta;
  phi(1, 5) = measurement_delta;
  phi(2, 6) = measurement_delta;
  phi(3, 7) = measurement_delta;
  Eigen::MatrixXd temp_p_k = p_k_dash;
  q_dash = 0.5 * ((phi * q * phi.transpose()) + q) * measurement_delta;
  temp_p_k = ((phi * temp_p_k * phi.transpose()) + q_dash).eval();
  Eigen::MatrixXd cov_mat = Eigen::MatrixXd::Identity(6, 6);
  cov_mat.block(0, 0, 3, 3) = temp_p_k.block(0, 0, 3, 3);
  cov_mat.block(3, 3, 3, 3) = temp_p_k.block(4, 4, 3, 3);

  return cov_mat;
}

Eigen::VectorXd
LinearInputModel::calculateInput(const Eigen::VectorXd &current_state) {
  Eigen::Matrix<double, 2, 2> w_to_b;
  w_to_b.setZero();
  w_to_b(0, 0) = cos(current_state(3));
  w_to_b(0, 1) = sin(current_state(3));
  w_to_b(1, 0) = -sin(current_state(3));
  w_to_b(1, 1) = cos(current_state(3));
  Eigen::MatrixXd b_to_w = w_to_b.inverse();
  Eigen::VectorXd v_in_body = w_to_b * current_state.block(4, 0, 2, 1);
  std::cout << " v in body " << v_in_body(0) << " " << v_in_body(1);
  Eigen::VectorXd drag_in_body;
  drag_in_body = -drag_multiplier * v_in_body;
  drag_in_body(0) = 0.0;
  Eigen::VectorXd drag_in_world = b_to_w * drag_in_body;
  std::cout << " vx: " << current_state(4);
  std::cout << " vy: " << current_state(5);
  std::cout << " drag x: " << drag_in_world(0);
  std::cout << " drag y: " << drag_in_world(1) << "\n";
  Eigen::VectorXd calculated_input = Eigen::VectorXd::Zero(STATE_COMPONENTS_);
  calculated_input(4) = drag_in_world(0);
  calculated_input(5) = drag_in_world(1);
  return calculated_input;
}
