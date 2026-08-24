#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <eigen3/Eigen/Dense>

#include <chrono>
#include <map>
#include <sstream>
#include <string>
using namespace Eigen;
using namespace std::chrono_literals;

class Clase_Publisher : public rclcpp::Node
{
public:
  Clase_Publisher()
  : rclcpp::Node("aplicar_fuerzas_dron")
  {
    pub_motor1 = this->create_publisher<std_msgs::msg::Float64>("motor/arr_iz", 10);
    pub_motor2 = this->create_publisher<std_msgs::msg::Float64>("motor/ab_iz", 10);
    pub_motor3 = this->create_publisher<std_msgs::msg::Float64>("motor/ab_der", 10);
    pub_motor4 = this->create_publisher<std_msgs::msg::Float64>("motor/arr_der", 10);
    objeto_timer =
      this->create_wall_timer(20ms, std::bind(&Clase_Publisher::aplicar_fuerzas, this));

    sub_fuerza_ = this->create_subscription<std_msgs::msg::Float64>(
      "control/tray/fuerza", 10,
      std::bind(&Clase_Publisher::callback_fuerza, this, std::placeholders::_1));
    sub_torque_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "control/tray/torque", 10,
      std::bind(&Clase_Publisher::callback_torque, this, std::placeholders::_1));

    // Declarar parámetro (tipo, nombre, valor por defecto)
    this->declare_parameter<double>("actuadores.conversor.fuerza2torque", 0.02);
    this->declare_parameter<double>("fisico.brazos.longitud", 0.25);
    this->declare_parameter<double>("fisico.brazos.grados", 45.0);
    this->declare_parameter<int64_t>("drone_id", 0);
    this->declare_parameter<bool>("debug_architecture_telemetry", false);

    drone_id_ = static_cast<uint32_t>(this->get_parameter("drone_id").as_int());
    debug_architecture_telemetry_ =
      this->get_parameter("debug_architecture_telemetry").as_bool();
    if (debug_architecture_telemetry_) {
      architecture_activity_pub_ =
        this->create_publisher<std_msgs::msg::String>(
        "/system_architecture/activity",
        rclcpp::QoS(64).best_effort());
    }

    fuerza2torque = this->get_parameter("actuadores.conversor.fuerza2torque").as_double();
    longitud_brazo = this->get_parameter("fisico.brazos.longitud").as_double();
    angulo_giro = this->get_parameter("fisico.brazos.grados").as_double();
    angulo_giro = angulo_giro * M_PI / 180.0;

    calcular_A_inv();
  }

private:
  void EmitArchitectureActivity()
  {
    if (!debug_architecture_telemetry_ || !architecture_activity_pub_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (architecture_last_emit_.time_since_epoch().count() != 0 &&
      now - architecture_last_emit_ < std::chrono::milliseconds(100))
    {
      return;
    }
    architecture_last_emit_ = now;
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"kind\":\"architecture_activity\","
      "\"edge_id\":\"dron_to_sim_motors\","
      "\"interface\":\"motor/*\","
      "\"interface_kind\":\"topic\","
      "\"source\":\"aplicar_fuerzas_dron\","
      "\"namespace\":\"" << this->get_namespace()
         << "\",\"drone_id\":" << drone_id_
         << ",\"timestamp\":" << this->get_clock()->now().seconds() << "}";
    message.data = json.str();
    architecture_activity_pub_->publish(message);
  }

  void callback_fuerza(const std_msgs::msg::Float64::SharedPtr msg)
  {
    fuerza_total = msg->data;
  }

  void callback_torque(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    torque << msg->vector.x, msg->vector.y, msg->vector.z;
  }


  void calcular_A_inv()
  {
    Eigen::Matrix4d A_{Eigen::Matrix4d::Zero()};

    A_.setZero();
    A_.row(0) << 1, 1, 1, 1;
    A_(1, 0) = longitud_brazo * std::cos(angulo_giro);
    A_(1, 1) = longitud_brazo * std::cos(angulo_giro);
    A_(1, 2) = -longitud_brazo * std::cos(angulo_giro);
    A_(1, 3) = -longitud_brazo * std::cos(angulo_giro);

    A_(2, 0) = -longitud_brazo * std::sin(angulo_giro);
    A_(2, 1) = longitud_brazo * std::sin(angulo_giro);
    A_(2, 2) = longitud_brazo * std::sin(angulo_giro);
    A_(2, 3) = -longitud_brazo * std::sin(angulo_giro);

    A_(3, 0) = -1.0 * fuerza2torque;
    A_(3, 1) = 1.0 * fuerza2torque;
    A_(3, 2) = -1.0 * fuerza2torque;
    A_(3, 3) = 1.0 * fuerza2torque;

    double cero = 1e-10;
    if (std::abs(A_.determinant()) < cero) {
      RCLCPP_WARN(get_logger(), "A casi singular; usando pseudoinversa.");
      Ainv_ = A_.completeOrthogonalDecomposition().pseudoInverse();
    } else {
      Ainv_ = A_.inverse();
    }
  }


  void aplicar_fuerzas()
  {
    Eigen::Vector4d U;
    U << fuerza_total, torque.x(), torque.y(), torque.z();

    Eigen::Vector4d f = Ainv_ * U;

    std_msgs::msg::Float64 msg;
    msg.data = f[0];
    pub_motor1->publish(msg);
    msg.data = f[1];
    pub_motor2->publish(msg);
    msg.data = f[2];
    pub_motor3->publish(msg);
    msg.data = f[3];
    pub_motor4->publish(msg);
    if (debug_architecture_telemetry_) {
      EmitArchitectureActivity();
    }

  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor1;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor2;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor3;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor4;
  rclcpp::TimerBase::SharedPtr objeto_timer;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_fuerza_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_torque_;

  double fuerza_total{0.0};
  Vector3d torque{Vector3d::Zero()};

  double fuerza2torque;
  double longitud_brazo;
  double angulo_giro;

  Eigen::Matrix4d Ainv_{Eigen::Matrix4d::Identity()};

  bool debug_architecture_telemetry_{false};
  uint32_t drone_id_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr architecture_activity_pub_;
  std::chrono::steady_clock::time_point architecture_last_emit_{};

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto objeto_nodo = std::make_shared<Clase_Publisher>();
  rclcpp::spin(objeto_nodo);

  rclcpp::shutdown();
  return 0;
}
