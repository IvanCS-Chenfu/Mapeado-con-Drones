#ifndef LIB_TRAY__GEN_TRAY_ELIPSE_HPP_
#define LIB_TRAY__GEN_TRAY_ELIPSE_HPP_

#include <array>
#include <cstddef>
#include <vector>

namespace lib_tray
{

// Estructura para obtener los parámetros de la trayectoria elipsoidal
struct CoefElipse
{
  double x0{0.0};
  double y0{0.0};
  double z0{0.0};
  double yaw0{0.0};

  double xc{0.0};
  double yc{0.0};
  double zc{0.0};

  double rx{0.0};
  double ry{0.0};

  double alpha{0.0};            // ángulo de orientación de la elipse
  double yaw_ref{0.0};          // yaw absoluto o offset respecto al centro

  double theta0{0.0};
  double tf{0.0};

  double longitud_total{0.0};

  // Perfil trapezoidal sobre la longitud de arco
  double t_a{0.0};
  double t1{0.0};
  double tc{0.0};
  double t2{0.0};
  double v_arco{0.0};
  double a_arco{0.0};
  bool triangular{false};

  bool yaw_absoluto{false};

  std::vector<double> tabla_theta;
  std::vector<double> tabla_s;
};

class GenTrayElipse
{
public:
  // Debido a que el constructor solo tiene un parámetro, el compilador puede convertir un número en "lib_tray::GenTrayElipse(<NUMERO>)"
  explicit GenTrayElipse(std::size_t N_ejes);

  void calcular_trayectoria(
    const std::vector<double> & posiciones_iniciales,
    const std::vector<double> & centro_elipse,
    const std::vector<double> & parametros_elipse,
    bool yaw_absoluto,
    double t_a);

  std::vector<std::array<double, 5>> evaluar(double t) const;

  void evaluar(double t, std::vector<std::array<double, 5>> & salida) const;

  std::size_t get_num_ejes() const;

  const CoefElipse & get_coeficientes() const;

private:
  // Calcula los parámetros de la trayectoria elipsoidal
  CoefElipse calcular_coeficientes(
    const std::vector<double> & posiciones_iniciales,
    const std::vector<double> & centro_elipse,
    const std::vector<double> & parametros_elipse,
    bool yaw_absoluto,
    double t_a) const;

  // Evalúa el perfil trapezoidal sobre la longitud de arco:
  // [s, s_dot, s_ddot, s_dddot, ratio]
  std::array<double, 5> evaluar_perfil_arco(const CoefElipse & c, double t) const;

  // Evalúa la trayectoria completa
  void evaluar_trayectoria(
    const CoefElipse & c, double t, std::vector<std::array<double,
    5>> & salida) const;

  double theta_desde_tiempo(const CoefElipse & c, double t) const;

  double ds_dtheta(const CoefElipse & c, double theta) const;

  double normalizar_angulo(double angulo) const;

  double posicion_yaw(const CoefElipse & c, double t) const;

  void comprobar_tamano_vector(
    const std::vector<double> & vector, const char * nombre_vector,
    std::size_t tamano) const;

  std::size_t N_ejes_{0};
  CoefElipse coeficientes_;
};

}  // namespace lib_tray

#endif  // LIB_TRAY__GEN_TRAY_ELIPSE_HPP_
