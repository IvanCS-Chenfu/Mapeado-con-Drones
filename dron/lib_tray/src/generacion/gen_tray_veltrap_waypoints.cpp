#include "lib_tray/gen_tray_veltrap_waypoints.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace lib_tray
{

GenTrayVelTrapWaypoints::GenTrayVelTrapWaypoints(std::size_t num_axes)
: num_axes_(num_axes)
{
  if (num_axes_ == 0U) {
    throw std::invalid_argument("num_axes debe ser mayor que 0");
  }
}

void GenTrayVelTrapWaypoints::calcular_trayectoria(
  const std::vector<double> & posiciones_iniciales,
  const std::vector<double> & velocidades_iniciales,
  const std::vector<std::vector<double>> & destinos,
  const std::vector<double> & velocidades_maximas,
  double tiempo_aceleracion)
{
  comprobar_vector(posiciones_iniciales, "posiciones_iniciales");
  comprobar_vector(velocidades_iniciales, "velocidades_iniciales");
  comprobar_vector(velocidades_maximas, "velocidades_maximas");
  if (destinos.empty()) {
    throw std::invalid_argument("se requiere al menos un destino");
  }
  if (tiempo_aceleracion <= 0.0) {
    throw std::invalid_argument("tiempo_aceleracion debe ser mayor que 0");
  }

  tramos_.clear();
  inicios_tramo_.clear();
  tiempos_llegada_.clear();
  tiempo_total_ = 0.0;

  std::vector<std::vector<double>> posiciones;
  posiciones.reserve(destinos.size() + 1U);
  posiciones.push_back(posiciones_iniciales);
  posiciones.insert(posiciones.end(), destinos.begin(), destinos.end());

  // La primera pasada conserva la lógica VelTrap para estimar sus tiempos,
  // sin aceptar un horario externo de misión.
  std::vector<double> tiempos_estimados{0.0};
  std::vector<double> previous_position = posiciones_iniciales;
  for (const auto & target : destinos) {
    GenTrayVelTrap estimate(num_axes_);
    estimate.calcular_trayectoria(
      previous_position, target, std::vector<double>(num_axes_, 0.0),
      std::vector<double>(num_axes_, 0.0), velocidades_maximas,
      tiempo_aceleracion, std::vector<bool>(num_axes_, true));
    double duration = 0.0;
    for (const auto & coefficient : estimate.get_coeficientes()) {
      duration = std::max(duration, coefficient.tf);
    }
    tiempos_estimados.push_back(tiempos_estimados.back() + duration);
    previous_position = target;
  }

  std::vector<std::vector<double>> velocidades(
    posiciones.size(), std::vector<double>(num_axes_, 0.0));
  velocidades.front() = velocidades_iniciales;
  for (std::size_t vertex = 1U; vertex + 1U < posiciones.size(); ++vertex) {
    const double before = tiempos_estimados[vertex] - tiempos_estimados[vertex - 1U];
    const double after = tiempos_estimados[vertex + 1U] - tiempos_estimados[vertex];
    for (std::size_t axis = 0U; axis < num_axes_; ++axis) {
      if (before <= 1e-9 || after <= 1e-9) {
        velocidades[vertex][axis] = 0.0;
        continue;
      }
      const double previous_slope = (posiciones[vertex][axis] - posiciones[vertex - 1U][axis]) /
        before;
      const double next_slope = (posiciones[vertex + 1U][axis] - posiciones[vertex][axis]) /
        after;
      velocidades[vertex][axis] = previous_slope * next_slope <= 0.0 ? 0.0 :
        (previous_slope * after + next_slope * before) / (before + after);
    }
  }

  previous_position = posiciones_iniciales;
  for (std::size_t index = 0U; index < destinos.size(); ++index) {
    GenTrayVelTrap tramo(num_axes_);
    tramo.calcular_trayectoria(
      previous_position, destinos[index], velocidades[index], velocidades[index + 1U],
      velocidades_maximas, tiempo_aceleracion, std::vector<bool>(num_axes_, true));
    double duration = 0.0;
    for (const auto & coefficient : tramo.get_coeficientes()) {
      duration = std::max(duration, coefficient.tf);
    }
    inicios_tramo_.push_back(tiempo_total_);
    tiempo_total_ += duration;
    tiempos_llegada_.push_back(tiempo_total_);
    tramos_.push_back(std::move(tramo));
    previous_position = destinos[index];
  }
}

std::vector<std::array<double, 5>> GenTrayVelTrapWaypoints::evaluar(double tiempo) const
{
  std::vector<std::array<double, 5>> salida(num_axes_);
  evaluar(tiempo, salida);
  return salida;
}

void GenTrayVelTrapWaypoints::evaluar(
  double tiempo, std::vector<std::array<double, 5>> & salida) const
{
  if (tramos_.empty()) {
    throw std::logic_error("la trayectoria VelTrap por waypoints no esta calculada");
  }
  const auto upper = std::upper_bound(tiempos_llegada_.begin(), tiempos_llegada_.end(), tiempo);
  const std::size_t segment = upper == tiempos_llegada_.end() ? tramos_.size() - 1U :
    static_cast<std::size_t>(upper - tiempos_llegada_.begin());
  tramos_[segment].evaluar(std::max(0.0, tiempo - inicios_tramo_[segment]), salida);
}

double GenTrayVelTrapWaypoints::get_tiempo_total() const
{
  return tiempo_total_;
}

const std::vector<double> & GenTrayVelTrapWaypoints::get_tiempos_llegada() const
{
  return tiempos_llegada_;
}

void GenTrayVelTrapWaypoints::comprobar_vector(
  const std::vector<double> & values, const char * name) const
{
  if (values.size() != num_axes_) {
    throw std::invalid_argument(
            std::string("El vector '") + name + "' debe tener tamaño " +
            std::to_string(num_axes_));
  }
}

}  // namespace lib_tray
