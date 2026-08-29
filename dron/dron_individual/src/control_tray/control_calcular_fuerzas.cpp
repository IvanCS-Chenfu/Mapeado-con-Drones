#include "rclcpp/rclcpp.hpp"
#include "dron_individual/action/tray_action.hpp"
#include "orbslam3_msgs/msg/navigation_state.hpp"

#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <chrono>
using namespace Eigen;
using namespace std::chrono_literals;

class Clase_Publisher : public rclcpp::Node
{
public:
  Clase_Publisher()
  : rclcpp::Node("control_calcular_fuerzas")
  {
    pub_torque_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "control/tray/torque",
      10);
    pub_fuerza_ = this->create_publisher<std_msgs::msg::Float64>("control/tray/fuerza", 10);
    objeto_timer = this->create_wall_timer(20ms, std::bind(&Clase_Publisher::enviar_fuerzas, this));

    sub_navigation_state_ =
      this->create_subscription<orbslam3_msgs::msg::NavigationState>(
      "orbslam/navigation_state", rclcpp::QoS(20).reliable(),
      std::bind(&Clase_Publisher::callback_navigation_state, this, std::placeholders::_1));
    sub_fb_ = this->create_subscription<dron_individual::action::TrayAction_FeedbackMessage>(
      "AccionTrayectoria/_action/feedback", 10,
      std::bind(&Clase_Publisher::callback_feedback, this, std::placeholders::_1));

    // Declarar parámetro (tipo, nombre, valor por defecto)
    this->declare_parameter<double>("fisico.total.masa", 1.4);
    this->declare_parameter<std::vector<double>>(
      "fisico.total.matriz_inercia", {1e-4, 1e-4, 1e-4,
        0.0, 0.0, 0.0});
    this->declare_parameter<double>("fisico.constante.gravedad", -9.81);
    this->declare_parameter<double>("control.fuerza.kp", 2.5);
    this->declare_parameter<double>("control.fuerza.kv", 5.0);
    this->declare_parameter<double>("control.torque.kr", 0.5);
    this->declare_parameter<double>("control.torque.kw", 0.5);
    this->declare_parameter<double>("navigation_state_timeout_sec", 0.5);
    this->declare_parameter<double>("control.source_handoff_duration_sec", 0.5);
    this->declare_parameter<bool>("debug_orb_control_state", false);

    // Obtener parámetro (decir tipo)
    m = this->get_parameter("fisico.total.masa").as_double();
    inercia = this->get_parameter("fisico.total.matriz_inercia").as_double_array();
    gravedad = this->get_parameter("fisico.constante.gravedad").as_double();
    Kp = this->get_parameter("control.fuerza.kp").as_double();
    Kv = this->get_parameter("control.fuerza.kv").as_double();
    Kr = this->get_parameter("control.torque.kr").as_double();
    Kw = this->get_parameter("control.torque.kw").as_double();
    navigation_state_timeout_sec_ =
      this->get_parameter("navigation_state_timeout_sec").as_double();
    angular_handoff_duration_sec_ = std::max(
      0.05, this->get_parameter("control.source_handoff_duration_sec").as_double());
    debug_orb_control_state_ =
      this->get_parameter("debug_orb_control_state").as_bool();

    snap_des.setZero();
  }

private:
  void callback_navigation_state(
    const orbslam3_msgs::msg::NavigationState::SharedPtr msg)
  {
    last_navigation_stamp_sec_ = rclcpp::Time(msg->header.stamp).seconds();
    last_navigation_receive_stamp_sec_ = this->get_clock()->now().seconds();
    current_pose_source_ = msg->pose_source;
    current_map_epoch_ = msg->map_epoch;
    current_reference_keyframe_id_ = msg->reference_keyframe_id;
    current_tracking_state_ = msg->tracking_state;
    current_sample_sequence_ = msg->sample_sequence;
    if (!msg->local_valid || !msg->local_continuity_valid || !msg->velocity_valid) {
      state_ready_ = false;
      if (debug_orb_control_state_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 200,
          "[F5H-CONTROL-STATE-INVALID] stamp=%.6f source=%u epoch=%lu sample=%lu "
          "tracking=%d ref_kf=%lu local_valid=%s continuity_valid=%s velocity_valid=%s",
          last_navigation_stamp_sec_, current_pose_source_,
          static_cast<unsigned long>(current_map_epoch_),
          static_cast<unsigned long>(current_sample_sequence_),
          current_tracking_state_,
          static_cast<unsigned long>(current_reference_keyframe_id_),
          msg->local_valid ? "true" : "false",
          msg->local_continuity_valid ? "true" : "false",
          msg->velocity_valid ? "true" : "false");
      }
      return;
    }
    x << msg->o_t_body.position.x, msg->o_t_body.position.y, msg->o_t_body.position.z;

    double x = msg->o_t_body.orientation.x;
    double y = msg->o_t_body.orientation.y;
    double z = msg->o_t_body.orientation.z;
    double w = msg->o_t_body.orientation.w;

    R_act << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
      2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
      2 * (x * z - w * y), 2 * (z * y + w * x), 1 - 2 * (x * x + y * y);
    x_dot << msg->velocity.linear.x, msg->velocity.linear.y, msg->velocity.linear.z;

    // La velocidad angular llega expresada en el frame local continuo.
    w_world_ << msg->velocity.angular.x, msg->velocity.angular.y, msg->velocity.angular.z;
    w_b = R_act.transpose() * w_world_;

    const auto current_pose_source = msg->pose_source;
    if (!pose_source_initialized_) {
      last_pose_source_ = current_pose_source;
      pose_source_initialized_ = true;
    } else if (current_pose_source != last_pose_source_) {
      // TODO FASE 6: retirar hold/handoff al eliminar GT_FALLBACK y su source lock.
      const bool entering_orb_from_fallback =
        last_pose_source_ ==
        orbslam3_msgs::msg::NavigationState::POSE_SOURCE_GT_FALLBACK &&
        current_pose_source == orbslam3_msgs::msg::NavigationState::POSE_SOURCE_ORB;
      if (entering_orb_from_fallback) {
        angular_handoff_pending_ = true;
        angular_handoff_active_ = false;
      } else {
        angular_handoff_pending_ = false;
        angular_handoff_active_ = false;
      }
      if (feedback_activado) {
        x_des = this->x;
        x_dot_des = x_dot;
        x_ddot_des.setZero();
        yaw_des = std::atan2(R_act(1, 0), R_act(0, 0));
        yaw_dot_des = w_world_.z();
        yaw_ddot_des = 0.0;
        jerk_des.setZero();
        snap_des.setZero();
      }
      RCLCPP_WARN(
        this->get_logger(),
        "[F5H-CONTROL-SOURCE-HOLD] previous=%u current=%u feedback_active=%s",
        static_cast<unsigned>(last_pose_source_),
        static_cast<unsigned>(current_pose_source),
        feedback_activado ? "true" : "false");
      last_pose_source_ = current_pose_source;
    }

    state_received_at_ = std::chrono::steady_clock::now();
    state_ready_ = true;
  }

  void callback_feedback(const dron_individual::action::TrayAction_FeedbackMessage::SharedPtr msg)
  {
    x_des << msg->feedback.x.data[0], msg->feedback.y.data[0], msg->feedback.z.data[0];
    x_dot_des << msg->feedback.x.data[1], msg->feedback.y.data[1], msg->feedback.z.data[1];
    x_ddot_des << msg->feedback.x.data[2], msg->feedback.y.data[2], msg->feedback.z.data[2];

    yaw_des = msg->feedback.yaw.data[0];
    yaw_dot_des = msg->feedback.yaw.data[1];
    yaw_ddot_des = msg->feedback.yaw.data[2];

    jerk_des << msg->feedback.x.data[3], msg->feedback.y.data[3], msg->feedback.z.data[3];

    if (angular_handoff_pending_ && state_ready_) {
      angular_handoff_rotation_ = R_act;
      angular_handoff_omega_body_ = w_b;
      angular_handoff_started_at_ = std::chrono::steady_clock::now();
      angular_handoff_pending_ = false;
      angular_handoff_active_ = true;
      angular_handoff_first_cycle_ = true;
      angular_handoff_log_stage_ = 0;
    }

    feedback_activado = true;
  }


  double mod(const Vector3d & v)
  {
    return std::sqrt(v.squaredNorm());
  }
  Vector3d vee(const Matrix3d & M)
  {
    // Para una matriz skew: [ 0 -z  y; z  0 -x; -y x 0 ] -> (x,y,z)
    return Vector3d(M(2, 1), M(0, 2), M(1, 0));
  }


  void enviar_fuerzas()
  {
    const bool state_fresh = state_ready_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - state_received_at_).count() <=
      navigation_state_timeout_sec_;
    if (feedback_activado && state_fresh) {
      // Fuerzas
      Vector3d ep = x - x_des;
      Vector3d ev = x_dot - x_dot_des;
      Vector3d g(0.0, 0.0, gravedad);

      Vector3d F_des = -(Kp * ep) - (Kv * ev) + m * (x_ddot_des - g);                   // Desde World
      Vector3d f = F_des;                                                               // Desde World
      Vector3d jerk_F = jerk_des * m;
      F_des = R_act.transpose() * F_des;                                                // Desde Cuerpo

      // Torques
      Matrix3d I = Matrix3d::Identity();
      // Todo desde World
      Vector3d c_yaw(std::cos(yaw_des), std::sin(yaw_des), 0.0);
      Vector3d c_yaw_dot = yaw_dot_des * Vector3d(-std::sin(yaw_des), std::cos(yaw_des), 0.0);
      Vector3d c_yaw_ddot = yaw_ddot_des *
        Vector3d(-std::sin(yaw_des), std::cos(yaw_des), 0.0) - yaw_dot_des * yaw_dot_des * Vector3d(
        std::cos(
          yaw_des), std::sin(yaw_des), 0.0);

      Vector3d eje_z = f / mod(f);
      Vector3d eje_z_dot = (I - eje_z * eje_z.transpose()) * (jerk_F / mod(f));
      Vector3d eje_z_ddot = (I - eje_z * eje_z.transpose()) *
        (snap_des / mod(f) - (jerk_F * (f.transpose() * jerk_F) / (mod(f) * mod(f) * mod(f)))) -
        eje_z_dot * (eje_z.transpose() * eje_z_dot);

      Vector3d h = eje_z.cross(c_yaw);
      Vector3d h_dot = eje_z_dot.cross(c_yaw) + eje_z.cross(c_yaw_dot);
      Vector3d h_ddot = eje_z_ddot.cross(c_yaw) + 2 * eje_z_dot.cross(c_yaw_dot) + eje_z.cross(
        c_yaw_ddot);

      Vector3d eje_y = h / mod(h);
      Vector3d eje_y_dot = (I - eje_y * eje_y.transpose()) * (h_dot / mod(h));
      Vector3d eje_y_ddot = (I - eje_y * eje_y.transpose()) *
        (h_ddot / mod(h) - (h_dot * (h.transpose() * h_dot) / (mod(h) * mod(h) * mod(h)))) -
        eje_y_dot * eje_y_dot.transpose() * eje_y;

      Vector3d eje_x = eje_y.cross(eje_z);
      Vector3d eje_x_dot = eje_y_dot.cross(eje_z) + eje_y.cross(eje_z_dot);
      Vector3d eje_x_ddot = eje_y_ddot.cross(eje_z) + 2 * eje_y_dot.cross(eje_z_dot) + eje_y.cross(
        eje_z_ddot);

      // Todo desde Cuerpo
      Matrix3d R_des;
      Matrix3d R_dot_des;
      Matrix3d R_ddot_des;

      R_des.col(0) = eje_x; R_des.col(1) = eje_y; R_des.col(2) = eje_z;
      R_dot_des.col(0) = eje_x_dot; R_dot_des.col(1) = eje_y_dot; R_dot_des.col(2) = eje_z_dot;
      R_ddot_des.col(0) = eje_x_ddot; R_ddot_des.col(1) = eje_y_ddot;
      R_ddot_des.col(2) = eje_z_ddot;


      Vector3d Omega_des = vee(R_des.transpose() * R_dot_des);
      if (angular_handoff_active_) {
        double linear_alpha = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - angular_handoff_started_at_).count() /
          angular_handoff_duration_sec_;
        if (angular_handoff_first_cycle_) {
          linear_alpha = 0.0;
          angular_handoff_started_at_ = std::chrono::steady_clock::now();
          angular_handoff_first_cycle_ = false;
        }
        linear_alpha = std::clamp(linear_alpha, 0.0, 1.0);
        angular_handoff_alpha_ =
          linear_alpha * linear_alpha * (3.0 - 2.0 * linear_alpha);

        const Matrix3d nominal_rotation = R_des;
        const Vector3d nominal_omega_world = nominal_rotation * Omega_des;
        const Vector3d initial_omega_world =
          angular_handoff_rotation_ * angular_handoff_omega_body_;
        Quaterniond initial_quaternion(angular_handoff_rotation_);
        Quaterniond nominal_quaternion(nominal_rotation);
        if (initial_quaternion.dot(nominal_quaternion) < 0.0) {
          nominal_quaternion.coeffs() *= -1.0;
        }
        R_des = initial_quaternion.slerp(
          angular_handoff_alpha_, nominal_quaternion).normalized().toRotationMatrix();
        const Vector3d blended_omega_world =
          (1.0 - angular_handoff_alpha_) * initial_omega_world +
          angular_handoff_alpha_ * nominal_omega_world;
        Omega_des = R_des.transpose() * blended_omega_world;
      }
      //Vector3d Omega_dot_des = vee(R_des.transpose()*R_ddot_des-(R_des.transpose()*R_dot_des)*(R_des.transpose()*R_dot_des));
      Vector3d Omega_dot_des(0, 0, 0);

      Vector3d er = vee(R_des.transpose() * R_act - R_act.transpose() * R_des) / 2;
      Vector3d ew = w_b - R_act.transpose() * R_des * Omega_des;


      Matrix3d J;
      J << inercia[0], inercia[3], inercia[4],
        inercia[3], inercia[1], inercia[5],
        inercia[4], inercia[5], inercia[2];

      const Vector3d tau_er = -Kr * er;
      const Vector3d tau_ew = -Kw * ew;
      const Vector3d tau_feedforward = J * R_act.transpose() * R_des * Omega_dot_des;
      const Vector3d tau_gyro = w_b.cross(J * w_b);
      const Vector3d tau_des = tau_er + tau_ew + tau_feedforward + tau_gyro;

      if (debug_orb_control_state_) {
        const double control_stamp = this->get_clock()->now().seconds();
        const Quaterniond q_act(R_act);
        const Quaterniond q_des(R_des);
        RCLCPP_INFO(
          this->get_logger(),
          "[F5H-PHASE-CONTROL] namespace=%s control_stamp=%.9f "
          "state_stamp=%.9f state_receive_stamp=%.9f source=%u epoch=%lu "
          "sample=%lu tracking=%d ref_kf=%lu "
          "r_act_q=(%.9f,%.9f,%.9f,%.9f) r_des_q=(%.9f,%.9f,%.9f,%.9f) "
          "omega_o=(%.9f,%.9f,%.9f) omega_body=(%.9f,%.9f,%.9f) "
          "omega_des_body=(%.9f,%.9f,%.9f) er=(%.9f,%.9f,%.9f) "
          "ew=(%.9f,%.9f,%.9f) tau_er=(%.9f,%.9f,%.9f) "
          "tau_ew=(%.9f,%.9f,%.9f) tau_feedforward=(%.9f,%.9f,%.9f) "
          "tau_gyro=(%.9f,%.9f,%.9f) tau_total=(%.9f,%.9f,%.9f) "
          "kr=%.9f kw=%.9f force=%.9f",
          this->get_namespace(), control_stamp, last_navigation_stamp_sec_,
          last_navigation_receive_stamp_sec_, current_pose_source_,
          static_cast<unsigned long>(current_map_epoch_),
          static_cast<unsigned long>(current_sample_sequence_),
          current_tracking_state_,
          static_cast<unsigned long>(current_reference_keyframe_id_),
          q_act.x(), q_act.y(), q_act.z(), q_act.w(), q_des.x(), q_des.y(),
          q_des.z(), q_des.w(), w_world_.x(), w_world_.y(), w_world_.z(),
          w_b.x(), w_b.y(), w_b.z(), Omega_des.x(), Omega_des.y(),
          Omega_des.z(), er.x(), er.y(), er.z(), ew.x(), ew.y(), ew.z(),
          tau_er.x(), tau_er.y(), tau_er.z(), tau_ew.x(), tau_ew.y(),
          tau_ew.z(), tau_feedforward.x(), tau_feedforward.y(),
          tau_feedforward.z(), tau_gyro.x(), tau_gyro.y(), tau_gyro.z(),
          tau_des.x(), tau_des.y(), tau_des.z(), Kr, Kw, F_des.z());
      }

      if (debug_orb_control_state_) {
        const double roll = std::atan2(R_act(2, 1), R_act(2, 2));
        const double pitch = std::asin(std::clamp(-R_act(2, 0), -1.0, 1.0));
        const double yaw = std::atan2(R_act(1, 0), R_act(0, 0));
        const double state_age_sec = last_navigation_stamp_sec_ > 0.0 ?
          std::max(0.0, this->get_clock()->now().seconds() - last_navigation_stamp_sec_) : -1.0;
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 100,
          "[F5H-CONTROL-DIAG] stamp=%.6f state_stamp=%.6f state_age_sec=%.6f "
          "source=%u epoch=%lu sample=%lu tracking=%d ref_kf=%lu "
          "position_error_norm=%.6f velocity_error_norm=%.6f "
          "attitude_error_norm=%.6f angular_velocity_error_norm=%.6f "
          "force=%.6f torque_norm=%.6f rpy=(%.6f,%.6f,%.6f) "
          "omega_body=(%.6f,%.6f,%.6f)",
          this->get_clock()->now().seconds(), last_navigation_stamp_sec_, state_age_sec,
          current_pose_source_, static_cast<unsigned long>(current_map_epoch_),
          static_cast<unsigned long>(current_sample_sequence_), current_tracking_state_,
          static_cast<unsigned long>(current_reference_keyframe_id_), ep.norm(), ev.norm(),
          er.norm(), ew.norm(), F_des.z(), tau_des.norm(), roll, pitch, yaw,
          w_b.x(), w_b.y(), w_b.z());
      }

      if (angular_handoff_active_) {
        const bool log_start = angular_handoff_log_stage_ == 0;
        const bool log_middle =
          angular_handoff_log_stage_ == 1 && angular_handoff_alpha_ >= 0.5;
        const bool log_end =
          angular_handoff_log_stage_ == 2 && angular_handoff_alpha_ >= 1.0;
        if (log_start || log_middle || log_end) {
          const char * event = log_start ? "start" : (log_middle ? "middle" : "end");
          RCLCPP_WARN(
            this->get_logger(),
            "[F5H-ANGULAR-HANDOFF] event=%s alpha=%.6f er_norm=%.6f "
            "ew_norm=%.6f force=%.6f torque_norm=%.6f",
            event, angular_handoff_alpha_, er.norm(), ew.norm(), F_des.z(), tau_des.norm());
          ++angular_handoff_log_stage_;
        }
        if (angular_handoff_alpha_ >= 1.0) {
          angular_handoff_active_ = false;
        }
      }


      // Enviar Fuerza y Torque
      std_msgs::msg::Float64 fuerza_deseada;
      fuerza_deseada.data = F_des.z();
      pub_fuerza_->publish(fuerza_deseada);

      geometry_msgs::msg::Vector3Stamped torque_deseado;
      torque_deseado.header.frame_id = "cuerpo";
      torque_deseado.vector.x = tau_des.x();
      torque_deseado.vector.y = tau_des.y();
      torque_deseado.vector.z = tau_des.z();

      pub_torque_->publish(torque_deseado);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_torque_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fuerza_;
  rclcpp::TimerBase::SharedPtr objeto_timer;

  rclcpp::Subscription<orbslam3_msgs::msg::NavigationState>::SharedPtr
    sub_navigation_state_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_fuerza_;
  rclcpp::Subscription<dron_individual::action::TrayAction_FeedbackMessage>::SharedPtr sub_fb_;

  Matrix3d R_act;
  Vector3d jerk_des;
  Vector3d snap_des;
  Vector3d w_b;
  Vector3d w_world_{Vector3d::Zero()};
  Vector3d f;
  Vector3d x;
  Vector3d x_dot;
  Vector3d x_ddot;
  Vector3d x_des;
  Vector3d x_dot_des;
  Vector3d x_ddot_des;

  bool feedback_activado = false;
  bool state_ready_ = false;
  bool pose_source_initialized_ = false;
  bool angular_handoff_pending_ = false;
  bool angular_handoff_active_ = false;
  bool angular_handoff_first_cycle_ = false;
  bool debug_orb_control_state_ = false;
  int angular_handoff_log_stage_ = 0;
  uint8_t last_pose_source_ = orbslam3_msgs::msg::NavigationState::POSE_SOURCE_INVALID;
  uint8_t current_pose_source_ = orbslam3_msgs::msg::NavigationState::POSE_SOURCE_INVALID;
  uint64_t current_map_epoch_ = 0;
  uint64_t current_reference_keyframe_id_ = 0;
  uint64_t current_sample_sequence_ = 0;
  int8_t current_tracking_state_ = -1;
  double last_navigation_stamp_sec_{0.0};
  double last_navigation_receive_stamp_sec_{0.0};
  double navigation_state_timeout_sec_{0.5};
  double angular_handoff_duration_sec_{0.5};
  double angular_handoff_alpha_{0.0};
  std::chrono::steady_clock::time_point state_received_at_;
  std::chrono::steady_clock::time_point angular_handoff_started_at_;
  Matrix3d angular_handoff_rotation_{Matrix3d::Identity()};
  Vector3d angular_handoff_omega_body_{Vector3d::Zero()};

  double yaw_des;
  double yaw_dot_des;
  double yaw_ddot_des;
  double m;
  std::vector<double> inercia;
  double gravedad;
  double Kp;
  double Kv;
  double Kr;
  double Kw;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto objeto_nodo = std::make_shared<Clase_Publisher>();
  rclcpp::spin(objeto_nodo);

  rclcpp::shutdown();
  return 0;
}
