#include "lib_tray/gen_tray_veltrap.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace lib_tray
{

GenTrayVelTrap::GenTrayVelTrap(std::size_t N_ejes)
: N_ejes_(N_ejes), coeficientes_(N_ejes)
{
  if (N_ejes_ == 0) {
    throw std::invalid_argument("N_ejes debe ser mayor que 0");
  }
}

void GenTrayVelTrap::calcular_trayectoria(
  const std::vector<double> & posiciones_iniciales,
  const std::vector<double> & posiciones_finales,
  const std::vector<double> & velocidades_iniciales,
  const std::vector<double> & velocidades_finales,
  const std::vector<double> & velocidades_maximas,
  double t_a,
  const std::vector<bool> & finales_absolutas)
{
  comprobar_tamano_vector(posiciones_iniciales, "posiciones_iniciales");
  comprobar_tamano_vector(posiciones_finales, "posiciones_finales");
  comprobar_tamano_vector(velocidades_iniciales, "velocidades_iniciales");
  comprobar_tamano_vector(velocidades_finales, "velocidades_finales");
  comprobar_tamano_vector(velocidades_maximas, "velocidades_maximas");
  comprobar_tamano_vector(finales_absolutas, "finales_absolutas");

  if (t_a <= 0.0) {
    throw std::invalid_argument("El tiempo de aceleracion t_a debe ser mayor que 0");
  }

  for (std::size_t i = 0; i < N_ejes_; ++i) {
    const double q0 = posiciones_iniciales[i];

    const double qf = finales_absolutas[i] ? posiciones_finales[i] : q0 + posiciones_finales[i];

    const double v0 = velocidades_iniciales[i];
    const double vf = velocidades_finales[i];
    const double vmax = velocidades_maximas[i];

    coeficientes_[i] = calcular_coeficientes(q0, qf, v0, vf, vmax, t_a);
  }
}

std::vector<std::array<double, 5>> GenTrayVelTrap::evaluar(double t) const
{
  std::vector<std::array<double, 5>> salida(N_ejes_);
  evaluar(t, salida);
  return salida;
}

void GenTrayVelTrap::evaluar(double t, std::vector<std::array<double, 5>> & salida) const
{
  if (salida.size() != N_ejes_) {
    salida.resize(N_ejes_);
  }

  for (std::size_t i = 0; i < N_ejes_; ++i) {
    salida[i] = evaluar_eje(coeficientes_[i], t);
  }
}


std::size_t GenTrayVelTrap::get_num_ejes() const
{
  return N_ejes_;
}

const std::vector<CoefTrapVel> & GenTrayVelTrap::get_coeficientes() const
{
  return coeficientes_;
}


// Para una variable, calcula los parámetros de su perfil trapezoidal
CoefTrapVel GenTrayVelTrap::calcular_coeficientes(
  double q0, double qf, double v0, double vf,
  double vmax, double t_a) const
{
  CoefTrapVel c;

  c.p0 = q0;
  c.pf = qf;

  const double distancia = std::abs(qf - q0);
  const double eps = 1e-9;

  if (distancia <= eps) {
    c.estacionario = true;
    c.triangular = false;

    c.s = 1.0;
    c.v_max = 0.0;
    c.a = 0.0;

    c.t1 = 0.0;
    c.tc = 0.0;
    c.t2 = 0.0;
    c.tf = 0.0;

    c.p1 = q0;
    c.p2 = q0;

    return c;
  }

  if (vmax <= 0.0) {
    throw std::invalid_argument(
            "La velocidad maxima vmax debe ser mayor que 0 para ejes no estacionarios");
  }

  if (qf > q0) {
    c.s = 1.0;
  } else {
    c.s = -1.0;
  }

  c.v_max = std::abs(vmax);
  c.a = c.v_max / t_a;

  // VelTrap evalúa el perfil en la dirección del desplazamiento y usa la
  // magnitud de las velocidades. Una velocidad residual contraria, o mayor
  // que la velocidad máxima proyectada de un eje casi estacionario, produce
  // tiempos negativos y perfiles que no llegan nunca al destino. En ese caso
  // se parte de reposo para que el controlador pueda corregir el error.
  const double v0_toward = c.s * v0;
  const double vf_toward = c.s * vf;
  const double v0_profile =
    v0_toward >= 0.0 && v0_toward <= c.v_max ? v0_toward : 0.0;
  const double vf_profile =
    vf_toward >= 0.0 && vf_toward <= c.v_max ? vf_toward : 0.0;
  c.v0 = c.s * v0_profile;
  c.vf = c.s * vf_profile;

  double x1 = (c.v_max * c.v_max - v0_profile * v0_profile) / (2.0 * c.a);
  double x2 = (c.v_max * c.v_max - vf_profile * vf_profile) / (2.0 * c.a);

  if (x1 + x2 >= distancia) {         // Triangular
    c.triangular = true;

    c.v_max =
      std::sqrt(
      std::max(
        0.0,
        c.a * distancia + 0.5 * (v0_profile * v0_profile + vf_profile * vf_profile)));

    c.t1 = (c.v_max - v0_profile) / c.a;
    c.t2 = (c.v_max - vf_profile) / c.a;
    c.tc = 0.0;

    const double dx1 = 0.5 * (v0_profile + c.v_max) * c.t1;
    c.p1 = q0 + c.s * dx1;
    c.p2 = c.p1;

    c.tf = c.t1 + c.t2;
  } else {                          // Trapezoidal
    c.triangular = false;

    c.t1 = (c.v_max - v0_profile) / c.a;
    c.tc = (distancia - x1 - x2) / c.v_max;
    c.t2 = (c.v_max - vf_profile) / c.a;

    const double dx1 = 0.5 * (v0_profile + c.v_max) * c.t1;
    c.p1 = q0 + c.s * dx1;
    c.p2 = c.p1 + c.s * (c.v_max * c.tc);

    c.tf = c.t1 + c.tc + c.t2;
  }

  return c;
}

// Para una variable, utiliza los parámetros para calcular las variables deseadas
std::array<double, 5> GenTrayVelTrap::evaluar_eje(const CoefTrapVel & c, double t) const
{
  std::array<double, 5> resultado{};

  if (c.estacionario || c.tf <= 0.0) {
    resultado[0] = c.p0;
    resultado[1] = 0.0;
    resultado[2] = 0.0;
    resultado[3] = 0.0;
    resultado[4] = 100.0;

    return resultado;
  }

  double tt = t;

  if (tt < 0.0) {
    tt = 0.0;
  } else if (tt > c.tf) {
    tt = c.tf;
  }

  double p;
  double v;
  double a;
  double j = 0.0;

  if (tt <= c.t1) {
    const double tau = tt;

    a = c.s * c.a;
    v = c.s * (std::abs(c.v0) + c.a * tau);
    p = c.p0 + c.s * (std::abs(c.v0) * tau + 0.5 * c.a * tau * tau);
  } else if (tt <= c.t1 + c.tc) {       // si es triangular -> c.tc = 0 -> no se da este caso
    const double tau = tt - c.t1;

    a = 0.0;
    v = c.s * c.v_max;
    p = c.p1 + c.s * (c.v_max * tau);
  } else {
    const double tau = tt - (c.t1 + c.tc);

    a = -c.s * c.a;
    v = c.s * (c.v_max - c.a * tau);
    p = c.p2 + c.s * (c.v_max * tau - 0.5 * c.a * tau * tau);
  }

  resultado[0] = p;

  if (t >= c.tf) {
    resultado[1] = 0.0;
    resultado[2] = 0.0;
    resultado[3] = 0.0;
  } else {
    resultado[1] = v;
    resultado[2] = a;
    resultado[3] = j;
  }

  resultado[4] = 100.0 * tt / c.tf;

  return resultado;
}


void GenTrayVelTrap::comprobar_tamano_vector(
  const std::vector<double> & vector,
  const char * nombre_vector) const
{
  if (vector.size() != N_ejes_) {
    throw std::invalid_argument(
            std::string(
              "El vector '") + nombre_vector + "' debe tener tamaño N_ejes = " +
            std::to_string(N_ejes_));
  }
}

void GenTrayVelTrap::comprobar_tamano_vector(
  const std::vector<bool> & vector,
  const char * nombre_vector) const
{
  if (vector.size() != N_ejes_) {
    throw std::invalid_argument(
            std::string(
              "El vector '") + nombre_vector + "' debe tener tamaño N_ejes = " +
            std::to_string(N_ejes_));
  }
}

}  // namespace lib_tray
