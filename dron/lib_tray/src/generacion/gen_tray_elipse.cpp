#include "lib_tray/gen_tray_elipse.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace lib_tray
{

GenTrayElipse::GenTrayElipse(std::size_t N_ejes)
: N_ejes_(N_ejes)
{
  if (N_ejes_ == 0) {
    throw std::invalid_argument("N_ejes debe ser mayor que 0");
  }

  if (N_ejes_ != 4) {
    throw std::invalid_argument("GenTrayElipse necesita N_ejes = 4: x, y, z, yaw");
  }
}

void GenTrayElipse::calcular_trayectoria(
  const std::vector<double> & posiciones_iniciales,
  const std::vector<double> & centro_elipse,
  const std::vector<double> & parametros_elipse,
  bool yaw_absoluto,
  double t_a)
{
  comprobar_tamano_vector(posiciones_iniciales, "posiciones_iniciales", 4);
  comprobar_tamano_vector(centro_elipse, "centro_elipse", 4);
  comprobar_tamano_vector(parametros_elipse, "parametros_elipse", 4);

  coeficientes_ = calcular_coeficientes(
    posiciones_iniciales, centro_elipse, parametros_elipse,
    yaw_absoluto, t_a);
}

std::vector<std::array<double, 5>> GenTrayElipse::evaluar(double t) const
{
  std::vector<std::array<double, 5>> salida(N_ejes_);
  evaluar(t, salida);
  return salida;
}

void GenTrayElipse::evaluar(double t, std::vector<std::array<double, 5>> & salida) const
{
  if (salida.size() != N_ejes_) {
    salida.resize(N_ejes_);
  }

  evaluar_trayectoria(coeficientes_, t, salida);
}


std::size_t GenTrayElipse::get_num_ejes() const
{
  return N_ejes_;
}

const CoefElipse & GenTrayElipse::get_coeficientes() const
{
  return coeficientes_;
}


// Calcula los parámetros de la trayectoria elipsoidal
CoefElipse GenTrayElipse::calcular_coeficientes(
  const std::vector<double> & posiciones_iniciales,
  const std::vector<double> & centro_elipse,
  const std::vector<double> & parametros_elipse,
  bool yaw_absoluto,
  double t_a) const
{
  constexpr double PI = 3.14159265358979323846;
  constexpr std::size_t N_TABLA = 2000;

  CoefElipse c;

  c.x0 = posiciones_iniciales[0];
  c.y0 = posiciones_iniciales[1];
  c.z0 = posiciones_iniciales[2];
  c.yaw0 = posiciones_iniciales[3];

  c.xc = centro_elipse[0];
  c.yc = centro_elipse[1];
  c.zc = centro_elipse[2];
  c.yaw_ref = centro_elipse[3];

  c.rx = std::abs(parametros_elipse[0]);
  c.ry = std::abs(parametros_elipse[1]);
  c.alpha = parametros_elipse[2];
  c.tf = parametros_elipse[3];

  c.yaw_absoluto = yaw_absoluto;

  if (c.rx <= 0.0) {
    throw std::invalid_argument("El radio rx debe ser mayor que 0");
  }

  if (c.ry <= 0.0) {
    throw std::invalid_argument("El radio ry debe ser mayor que 0");
  }

  if (c.tf <= 0.0) {
    throw std::invalid_argument("El tiempo total tf debe ser mayor que 0");
  }

  const double ca = std::cos(c.alpha);
  const double sa = std::sin(c.alpha);

  const double dx = c.x0 - c.xc;
  const double dy = c.y0 - c.yc;

  // Transformación de world al marco local de la elipse
  const double x_local = ca * dx + sa * dy;
  const double y_local = -sa * dx + ca * dy;

  if (std::abs(x_local) <= 1e-9 && std::abs(y_local) <= 1e-9) {
    c.theta0 = 0.0;
  } else {
    c.theta0 = std::atan2(y_local / c.ry, x_local / c.rx);
  }

  c.tabla_theta.resize(N_TABLA + 1);
  c.tabla_s.resize(N_TABLA + 1);

  c.tabla_theta[0] = 0.0;
  c.tabla_s[0] = 0.0;

  for (std::size_t i = 1; i <= N_TABLA; ++i) {
    const double theta_anterior = c.theta0 + 2.0 * PI * static_cast<double>(i - 1) /
      static_cast<double>(N_TABLA);
    const double theta_actual = c.theta0 + 2.0 * PI * static_cast<double>(i) /
      static_cast<double>(N_TABLA);

    const double dtheta = theta_actual - theta_anterior;
    const double theta_medio = 0.5 * (theta_anterior + theta_actual);

    const double ds = ds_dtheta(c, theta_medio) * dtheta;

    c.tabla_theta[i] = 2.0 * PI * static_cast<double>(i) / static_cast<double>(N_TABLA);
    c.tabla_s[i] = c.tabla_s[i - 1] + ds;
  }

  c.longitud_total = c.tabla_s.back();

  // Perfil trapezoidal de velocidad sobre la longitud de arco.
  // La distancia a recorrer es c.longitud_total y el tiempo total es c.tf.
  if (t_a <= 0.0) {
    throw std::invalid_argument("El tiempo de aceleracion t_a debe ser mayor que 0");
  }

  c.t_a = t_a;

  // Si no cabe un trapezoidal completo, se genera triangular.
  if (2.0 * c.t_a >= c.tf) {
    c.triangular = true;

    c.t1 = 0.5 * c.tf;
    c.tc = 0.0;
    c.t2 = 0.5 * c.tf;

    c.v_arco = 2.0 * c.longitud_total / c.tf;
    c.a_arco = c.v_arco / c.t1;
  } else {
    c.triangular = false;

    c.t1 = c.t_a;
    c.tc = c.tf - 2.0 * c.t_a;
    c.t2 = c.t_a;

    // Área del perfil trapezoidal:
    // L = v_arco * (tf - t_a)
    c.v_arco = c.longitud_total / (c.tf - c.t_a);
    c.a_arco = c.v_arco / c.t_a;
  }

  return c;
}

// Evalúa la trayectoria completa
void GenTrayElipse::evaluar_trayectoria(
  const CoefElipse & c, double t,
  std::vector<std::array<double, 5>> & salida) const
{
  constexpr double PI = 3.14159265358979323846;

  double tt = t;

  if (tt < 0.0) {
    tt = 0.0;
  } else if (tt > c.tf) {
    tt = c.tf;
  }

  const auto perfil_arco = evaluar_perfil_arco(c, tt);

  const double s_arco = perfil_arco[0];
  const double v_arco = perfil_arco[1];
  const double a_arco = perfil_arco[2];
  const double j_arco = perfil_arco[3];

  const double ratio = s_arco / c.longitud_total;
  const double theta = theta_desde_tiempo(c, tt);

  const double ca = std::cos(c.alpha);
  const double sa = std::sin(c.alpha);

  const double ct = std::cos(theta);
  const double st = std::sin(theta);

  const double x_local = c.rx * ct;
  const double y_local = c.ry * st;

  const double x = c.xc + x_local * ca - y_local * sa;
  const double y = c.yc + x_local * sa + y_local * ca;

  const double z_final = c.z0 + 2.0 * (c.zc - c.z0);
  const double dz_total = z_final - c.z0;

  const double z = c.z0 + dz_total * ratio;

  const double D = ds_dtheta(c, theta);

  double theta_dot = 0.0;
  double theta_ddot = 0.0;
  double theta_dddot = 0.0;

  if (D > 1e-12) {
    const double rx2 = c.rx * c.rx;
    const double ry2 = c.ry * c.ry;
    const double B = rx2 - ry2;

    const double D1 = B * st * ct / D;
    const double D2 = B * std::cos(2.0 * theta) / D - (B * B * st * st * ct * ct) / (D * D * D);

    // s_dot = v_arco
    // s_ddot = a_arco
    // s_dddot = j_arco
    theta_dot = v_arco / D;

    theta_ddot = a_arco / D - (v_arco * v_arco * D1) / (D * D * D);

    theta_dddot = j_arco / D -
      (3.0 * v_arco * a_arco * D1) / (D * D * D) -
      v_arco * v_arco * v_arco *
      ((D2 / (D * D * D * D)) - (3.0 * D1 * D1 / (D * D * D * D * D)));
  }

  const double dxl_dtheta = -c.rx * st;
  const double dyl_dtheta = c.ry * ct;

  const double d2xl_dtheta2 = -c.rx * ct;
  const double d2yl_dtheta2 = -c.ry * st;

  const double d3xl_dtheta3 = c.rx * st;
  const double d3yl_dtheta3 = -c.ry * ct;

  const double vx_local = dxl_dtheta * theta_dot;
  const double vy_local = dyl_dtheta * theta_dot;

  const double ax_local = d2xl_dtheta2 * theta_dot * theta_dot + dxl_dtheta * theta_ddot;
  const double ay_local = d2yl_dtheta2 * theta_dot * theta_dot + dyl_dtheta * theta_ddot;

  const double jx_local = d3xl_dtheta3 * theta_dot * theta_dot * theta_dot +
    3.0 * d2xl_dtheta2 * theta_dot * theta_ddot +
    dxl_dtheta * theta_dddot;

  const double jy_local = d3yl_dtheta3 * theta_dot * theta_dot * theta_dot +
    3.0 * d2yl_dtheta2 * theta_dot * theta_ddot +
    dyl_dtheta * theta_dddot;

  double vx = vx_local * ca - vy_local * sa;
  double vy = vx_local * sa + vy_local * ca;

  double ax = ax_local * ca - ay_local * sa;
  double ay = ax_local * sa + ay_local * ca;

  double jx = jx_local * ca - jy_local * sa;
  double jy = jx_local * sa + jy_local * ca;

  double vz = dz_total * v_arco / c.longitud_total;
  double az = dz_total * a_arco / c.longitud_total;
  double jz = dz_total * j_arco / c.longitud_total;

  double yaw = posicion_yaw(c, tt);
  double vyaw = 0.0;
  double ayaw = 0.0;
  double jyaw = 0.0;

  if (!c.yaw_absoluto) {
    const double h = std::min(1e-3, c.tf * 1e-3);

    double t_menos = tt - h;
    double t_mas = tt + h;

    if (t_menos < 0.0) {
      t_menos = 0.0;
    }

    if (t_mas > c.tf) {
      t_mas = c.tf;
    }

    if (t_mas > t_menos) {
      const double yaw_menos = posicion_yaw(c, t_menos);
      const double yaw_mas = posicion_yaw(c, t_mas);

      vyaw = normalizar_angulo(yaw_mas - yaw_menos) / (t_mas - t_menos);
    }

    if (tt > h && tt < c.tf - h) {
      const double yaw_menos = posicion_yaw(c, tt - h);
      const double yaw_centro = posicion_yaw(c, tt);
      const double yaw_mas = posicion_yaw(c, tt + h);

      const double dyaw_1 = normalizar_angulo(yaw_centro - yaw_menos);
      const double dyaw_2 = normalizar_angulo(yaw_mas - yaw_centro);

      ayaw = (dyaw_2 - dyaw_1) / (h * h);
    }

    jyaw = 0.0;
  }

  if (t >= c.tf) {
    vx = 0.0;
    vy = 0.0;
    vz = 0.0;
    vyaw = 0.0;

    ax = 0.0;
    ay = 0.0;
    az = 0.0;
    ayaw = 0.0;

    jx = 0.0;
    jy = 0.0;
    jz = 0.0;
    jyaw = 0.0;
  }

  salida[0][0] = x;
  salida[0][1] = vx;
  salida[0][2] = ax;
  salida[0][3] = jx;
  salida[0][4] = 100.0 * ratio;

  salida[1][0] = y;
  salida[1][1] = vy;
  salida[1][2] = ay;
  salida[1][3] = jy;
  salida[1][4] = 100.0 * ratio;

  salida[2][0] = z;
  salida[2][1] = vz;
  salida[2][2] = az;
  salida[2][3] = jz;
  salida[2][4] = 100.0 * ratio;

  salida[3][0] = yaw;
  salida[3][1] = vyaw;
  salida[3][2] = ayaw;
  salida[3][3] = jyaw;
  salida[3][4] = 100.0 * ratio;

  (void)PI;
}


std::array<double, 5> GenTrayElipse::evaluar_perfil_arco(const CoefElipse & c, double t) const
{
  std::array<double, 5> resultado{};

  double tt = t;

  if (tt < 0.0) {
    tt = 0.0;
  } else if (tt > c.tf) {
    tt = c.tf;
  }

  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
  double j = 0.0;

  if (tt <= c.t1) {
    // Fase de aceleración
    const double tau = tt;

    a = c.a_arco;
    v = c.a_arco * tau;
    s = 0.5 * c.a_arco * tau * tau;
  } else if (tt <= c.t1 + c.tc) {
    // Fase de velocidad constante
    const double tau = tt - c.t1;

    a = 0.0;
    v = c.v_arco;
    s = 0.5 * c.a_arco * c.t1 * c.t1 + c.v_arco * tau;
  } else {
    // Fase de desaceleración
    const double tau = tt - (c.t1 + c.tc);

    const double s1 = 0.5 * c.a_arco * c.t1 * c.t1;
    const double s2 = c.v_arco * c.tc;

    a = -c.a_arco;
    v = c.v_arco - c.a_arco * tau;
    s = s1 + s2 + c.v_arco * tau - 0.5 * c.a_arco * tau * tau;
  }

  if (t >= c.tf) {
    s = c.longitud_total;
    v = 0.0;
    a = 0.0;
    j = 0.0;
  }

  resultado[0] = s;
  resultado[1] = v;
  resultado[2] = a;
  resultado[3] = j;
  resultado[4] = 100.0 * s / c.longitud_total;

  return resultado;
}


double GenTrayElipse::theta_desde_tiempo(const CoefElipse & c, double t) const
{
  constexpr double PI = 3.14159265358979323846;

  double tt = t;

  if (tt < 0.0) {
    tt = 0.0;
  } else if (tt > c.tf) {
    tt = c.tf;
  }

  const auto perfil_arco = evaluar_perfil_arco(c, tt);
  const double s_objetivo = perfil_arco[0];

  auto it = std::lower_bound(c.tabla_s.begin(), c.tabla_s.end(), s_objetivo);

  if (it == c.tabla_s.begin()) {
    return c.theta0;
  }

  if (it == c.tabla_s.end()) {
    return c.theta0 + 2.0 * PI;
  }

  const std::size_t idx = static_cast<std::size_t>(std::distance(c.tabla_s.begin(), it));

  const double s0 = c.tabla_s[idx - 1];
  const double s1 = c.tabla_s[idx];

  const double theta0 = c.tabla_theta[idx - 1];
  const double theta1 = c.tabla_theta[idx];

  double k = 0.0;

  if (std::abs(s1 - s0) > 1e-12) {
    k = (s_objetivo - s0) / (s1 - s0);
  }

  return c.theta0 + theta0 + k * (theta1 - theta0);
}

double GenTrayElipse::ds_dtheta(const CoefElipse & c, double theta) const
{
  const double st = std::sin(theta);
  const double ct = std::cos(theta);

  return std::sqrt(c.rx * c.rx * st * st + c.ry * c.ry * ct * ct);
}

double GenTrayElipse::normalizar_angulo(double angulo) const
{
  constexpr double PI = 3.14159265358979323846;

  while (angulo > PI) {
    angulo -= 2.0 * PI;
  }

  while (angulo < -PI) {
    angulo += 2.0 * PI;
  }

  return angulo;
}

double GenTrayElipse::posicion_yaw(const CoefElipse & c, double t) const
{
  double tt = t;

  if (tt < 0.0) {
    tt = 0.0;
  } else if (tt > c.tf) {
    tt = c.tf;
  }

  if (c.yaw_absoluto) {
    return normalizar_angulo(c.yaw_ref);
  }

  const double ratio = tt / c.tf;
  const double theta = theta_desde_tiempo(c, tt);

  const double ca = std::cos(c.alpha);
  const double sa = std::sin(c.alpha);

  const double x_local = c.rx * std::cos(theta);
  const double y_local = c.ry * std::sin(theta);

  const double x = c.xc + x_local * ca - y_local * sa;
  const double y = c.yc + x_local * sa + y_local * ca;

  const double yaw_centro = std::atan2(c.yc - y, c.xc - x);

  (void)ratio;

  return normalizar_angulo(yaw_centro + c.yaw_ref);
}


void GenTrayElipse::comprobar_tamano_vector(
  const std::vector<double> & vector,
  const char * nombre_vector, std::size_t tamano) const
{
  if (vector.size() != tamano) {
    throw std::invalid_argument(
            std::string(
              "El vector '") + nombre_vector + "' debe tener tamaño " + std::to_string(tamano));
  }
}

}  // namespace lib_tray
