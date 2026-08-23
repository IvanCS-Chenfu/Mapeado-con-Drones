#include "lib_tray/gen_tray_pol3.hpp"

#include <stdexcept>
#include <string>

namespace lib_tray
{

GenTrayPol3::GenTrayPol3(std::size_t N_ejes)
: N_ejes_(N_ejes), coeficientes_(N_ejes)
{
  if (N_ejes_ == 0) {
    throw std::invalid_argument("N_ejes debe ser mayor que 0");
  }
}

void GenTrayPol3::calcular_trayectoria(
  const std::vector<double> & posiciones_iniciales,
  const std::vector<double> & posiciones_finales,
  const std::vector<double> & velocidades_iniciales,
  const std::vector<double> & velocidades_finales,
  const std::vector<double> & tiempos_finales,
  const std::vector<bool> & finales_absolutas)
{
  comprobar_tamano_vector(posiciones_iniciales, "posiciones_iniciales");
  comprobar_tamano_vector(posiciones_finales, "posiciones_finales");
  comprobar_tamano_vector(velocidades_iniciales, "velocidades_iniciales");
  comprobar_tamano_vector(velocidades_finales, "velocidades_finales");
  comprobar_tamano_vector(tiempos_finales, "tiempos_finales");
  comprobar_tamano_vector(finales_absolutas, "finales_absolutas");

  for (std::size_t i = 0; i < N_ejes_; ++i) {
    const double q0 = posiciones_iniciales[i];

    const double qf = finales_absolutas[i] ? posiciones_finales[i] : q0 + posiciones_finales[i];

    const double v0 = velocidades_iniciales[i];
    const double vf = velocidades_finales[i];
    const double tf = tiempos_finales[i];

    coeficientes_[i] = calcular_coeficientes(q0, qf, v0, vf, tf);
  }
}

std::vector<std::array<double, 5>> GenTrayPol3::evaluar(double t) const
{
  std::vector<std::array<double, 5>> salida(N_ejes_);
  evaluar(t, salida);
  return salida;
}

void GenTrayPol3::evaluar(double t, std::vector<std::array<double, 5>> & salida) const      // Esta es la principal. Necesita un {std::vector<std::array<double, 5>> salida(N_ejes_);}
{
  if (salida.size() != N_ejes_) {
    salida.resize(N_ejes_);
  }

  for (std::size_t i = 0; i < N_ejes_; ++i) {
    salida[i] = evaluar_eje(coeficientes_[i], t);
  }
}


std::size_t GenTrayPol3::get_num_ejes() const
{
  return N_ejes_;
}

const std::vector<CoefCubic> & GenTrayPol3::get_coeficientes() const
{
  return coeficientes_;
}


// Para una variable, calcula los coeficientes de su polinomio cúbico
CoefCubic GenTrayPol3::calcular_coeficientes(
  double q0, double qf, double v0, double vf,
  double tf) const
{
  if (tf <= 0.0) {
    throw std::invalid_argument("El tiempo final tf debe ser mayor que 0");
  }

  const double tf2 = tf * tf;
  const double tf3 = tf2 * tf;

  CoefCubic c;

  c.tf = tf;
  c.a0 = q0;
  c.a1 = v0;
  c.a2 = (3.0 * (qf - q0) - (2.0 * v0 + vf) * tf) / tf2;
  c.a3 = (2.0 * (q0 - qf) + (v0 + vf) * tf) / tf3;

  return c;
}

// Para una variable, utiliza los coeficientes para calcular las variables deseadas
std::array<double, 5> GenTrayPol3::evaluar_eje(const CoefCubic & c, double t) const
{
  std::array<double, 5> resultado{};

  double tt = t;

  if (tt < 0.0) {
    tt = 0.0;
  } else if (tt > c.tf) {
    tt = c.tf;
  }

  const double tt2 = tt * tt;
  const double tt3 = tt2 * tt;

  resultado[0] = c.a0 + c.a1 * tt + c.a2 * tt2 + c.a3 * tt3;

  if (t >= c.tf) {
    resultado[1] = 0.0;
    resultado[2] = 0.0;
    resultado[3] = 0.0;
  } else {
    resultado[1] = c.a1 + 2.0 * c.a2 * tt + 3.0 * c.a3 * tt2;
    resultado[2] = 2.0 * c.a2 + 6.0 * c.a3 * tt;
    resultado[3] = 6.0 * c.a3;
  }

  resultado[4] = 100.0 * tt / c.tf;

  return resultado;
}


void GenTrayPol3::comprobar_tamano_vector(
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

void GenTrayPol3::comprobar_tamano_vector(
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
