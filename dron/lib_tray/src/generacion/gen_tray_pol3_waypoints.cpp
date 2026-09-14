#include "lib_tray/gen_tray_pol3_waypoints.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace lib_tray
{

GenTrayPol3Waypoints::GenTrayPol3Waypoints(std::size_t num_axes, double waypoint_blend_sec)
: num_axes_(num_axes), waypoint_blend_sec_(waypoint_blend_sec)
{
  if (num_axes_ == 0U) {
    throw std::invalid_argument("num_axes debe ser mayor que 0");
  }
  if (!std::isfinite(waypoint_blend_sec_) || waypoint_blend_sec_ <= 0.0) {
    throw std::invalid_argument("waypoint_blend_sec debe ser finito y mayor que 0");
  }
}

void GenTrayPol3Waypoints::calcular_trayectoria(
  const std::vector<double> & posiciones_iniciales,
  const std::vector<double> & velocidades_iniciales,
  const std::vector<std::vector<double>> & destinos,
  const std::vector<double> & tiempos_llegada)
{
  if (destinos.size() != tiempos_llegada.size()) {
    throw std::invalid_argument("destinos y tiempos_llegada deben tener el mismo tamaño");
  }
  std::vector<std::vector<double>> tiempos_por_eje;
  tiempos_por_eje.reserve(tiempos_llegada.size());
  for (const double tiempo : tiempos_llegada) {
    tiempos_por_eje.emplace_back(num_axes_, tiempo);
  }
  calcular_impl(posiciones_iniciales, velocidades_iniciales, destinos, tiempos_por_eje);
}

void GenTrayPol3Waypoints::calcular_trayectoria_por_eje(
  const std::vector<double> & posiciones_iniciales,
  const std::vector<double> & velocidades_iniciales,
  const std::vector<std::vector<double>> & destinos,
  const std::vector<std::vector<double>> & tiempos_llegada_por_eje)
{
  calcular_impl(
    posiciones_iniciales, velocidades_iniciales, destinos, tiempos_llegada_por_eje);
}

void GenTrayPol3Waypoints::calcular_impl(
  const std::vector<double> & posiciones_iniciales,
  const std::vector<double> & velocidades_iniciales,
  const std::vector<std::vector<double>> & destinos,
  const std::vector<std::vector<double>> & tiempos_llegada_por_eje)
{
  comprobar_vector(posiciones_iniciales, "posiciones_iniciales");
  comprobar_vector(velocidades_iniciales, "velocidades_iniciales");
  if (destinos.empty() || destinos.size() != tiempos_llegada_por_eje.size()) {
    throw std::invalid_argument("se requiere un destino y sus tiempos de llegada");
  }
  for (std::size_t index = 0; index < destinos.size(); ++index) {
    comprobar_vector(destinos[index], "destino");
    comprobar_vector(tiempos_llegada_por_eje[index], "tiempo_llegada_por_eje");
  }

  tramos_.clear();
  inicios_tramo_.clear();
  finales_tramo_.clear();
  tiempo_total_ = 0.0;

  const auto append_segment = [this](
    const std::vector<double> & initial_position,
    const std::vector<double> & final_position,
    const std::vector<double> & initial_velocity,
    const std::vector<double> & final_velocity,
    double duration) {
      if (!std::isfinite(duration) || duration <= 0.0) {
        throw std::invalid_argument("un tramo Pol3 debe tener duración positiva");
      }
      GenTrayPol3 segment(num_axes_);
      segment.calcular_trayectoria(
        initial_position, final_position, initial_velocity, final_velocity,
        std::vector<double>(num_axes_, duration), std::vector<bool>(num_axes_, true));
      inicios_tramo_.push_back(tiempo_total_);
      tiempo_total_ += duration;
      finales_tramo_.push_back(tiempo_total_);
      tramos_.push_back(std::move(segment));
    };

  // Un destino conserva exactamente los tiempos independientes por eje del perfil legacy.
  if (destinos.size() == 1U) {
    std::vector<double> durations(num_axes_);
    double duration = 0.0;
    for (std::size_t axis = 0U; axis < num_axes_; ++axis) {
      const double arrival = tiempos_llegada_por_eje.front()[axis];
      if (!std::isfinite(arrival) || arrival <= 0.0) {
        throw std::invalid_argument("los tiempos de llegada deben aumentar estrictamente");
      }
      durations[axis] = arrival;
      duration = std::max(duration, arrival);
    }
    GenTrayPol3 segment(num_axes_);
    segment.calcular_trayectoria(
      posiciones_iniciales, destinos.front(), velocidades_iniciales,
      std::vector<double>(num_axes_, 0.0), durations,
      std::vector<bool>(num_axes_, true));
    tramos_.push_back(std::move(segment));
    inicios_tramo_.push_back(0.0);
    finales_tramo_.push_back(duration);
    tiempo_total_ = duration;
    return;
  }

  std::vector<double> arrivals;
  arrivals.reserve(destinos.size());
  double previous_arrival = 0.0;
  for (const auto & arrival_per_axis : tiempos_llegada_por_eje) {
    const double arrival = arrival_per_axis.front();
    if (!std::isfinite(arrival) || arrival <= previous_arrival) {
      throw std::invalid_argument("los tiempos de llegada deben aumentar estrictamente");
    }
    for (std::size_t axis = 1U; axis < num_axes_; ++axis) {
      if (std::abs(arrival_per_axis[axis] - arrival) > 1e-9) {
        throw std::invalid_argument(
                "los waypoints multi-destino requieren un tiempo común por eje");
      }
    }
    arrivals.push_back(arrival);
    previous_arrival = arrival;
  }

  std::vector<std::vector<double>> positions;
  positions.reserve(destinos.size() + 1U);
  positions.push_back(posiciones_iniciales);
  positions.insert(positions.end(), destinos.begin(), destinos.end());
  const std::vector<double> zero_velocity(num_axes_, 0.0);

  std::vector<GenTrayPol3> nominal_segments;
  nominal_segments.reserve(destinos.size());
  std::vector<double> durations;
  durations.reserve(destinos.size());
  for (std::size_t index = 0U; index < destinos.size(); ++index) {
    const double segment_start = index == 0U ? 0.0 : arrivals[index - 1U];
    const double duration = arrivals[index] - segment_start;
    const double left_trim = index == 0U ? 0.0 : waypoint_blend_sec_;
    const double right_trim = index + 1U == destinos.size() ? 0.0 : waypoint_blend_sec_;
    if (duration <= left_trim + right_trim) {
      throw std::invalid_argument(
              "la duración de un tramo no deja espacio para los empalmes C1");
    }
    GenTrayPol3 nominal(num_axes_);
    nominal.calcular_trayectoria(
      positions[index], positions[index + 1U],
      index == 0U ? velocidades_iniciales : zero_velocity, zero_velocity,
      std::vector<double>(num_axes_, duration), std::vector<bool>(num_axes_, true));
    nominal_segments.push_back(std::move(nominal));
    durations.push_back(duration);
  }

  const auto sample_state = [this](const GenTrayPol3 & segment, double time) {
      const auto values = segment.evaluar(time);
      std::vector<double> position(num_axes_);
      std::vector<double> velocity(num_axes_);
      for (std::size_t axis = 0U; axis < num_axes_; ++axis) {
        position[axis] = values[axis][0];
        velocity[axis] = values[axis][1];
      }
      return std::make_pair(position, velocity);
    };

  for (std::size_t index = 0U; index < nominal_segments.size(); ++index) {
    const double left_trim = index == 0U ? 0.0 : waypoint_blend_sec_;
    const double right_trim =
      index + 1U == nominal_segments.size() ? 0.0 : waypoint_blend_sec_;
    const auto start = sample_state(nominal_segments[index], left_trim);
    const auto finish = sample_state(nominal_segments[index], durations[index] - right_trim);
    append_segment(
      start.first, finish.first, start.second, finish.second,
      durations[index] - left_trim - right_trim);

    if (index + 1U < nominal_segments.size()) {
      const auto next = sample_state(nominal_segments[index + 1U], waypoint_blend_sec_);
      append_segment(
        finish.first, next.first, finish.second, next.second, 2.0 * waypoint_blend_sec_);
    }
  }
}

std::vector<std::array<double, 5>> GenTrayPol3Waypoints::evaluar(double tiempo) const
{
  std::vector<std::array<double, 5>> salida(num_axes_);
  evaluar(tiempo, salida);
  return salida;
}

void GenTrayPol3Waypoints::evaluar(
  double tiempo, std::vector<std::array<double, 5>> & salida) const
{
  const std::size_t segment = get_indice_tramo(tiempo);
  tramos_[segment].evaluar(
    std::clamp(
      tiempo - inicios_tramo_[segment], 0.0,
      finales_tramo_[segment] - inicios_tramo_[segment]), salida);
}

std::size_t GenTrayPol3Waypoints::get_num_ejes() const
{
  return num_axes_;
}

double GenTrayPol3Waypoints::get_tiempo_total() const
{
  return tiempo_total_;
}

std::size_t GenTrayPol3Waypoints::get_num_tramos() const
{
  return tramos_.size();
}

std::size_t GenTrayPol3Waypoints::get_indice_tramo(double tiempo) const
{
  if (tramos_.empty()) {
    throw std::logic_error("la trayectoria por waypoints no esta calculada");
  }
  const auto upper = std::upper_bound(finales_tramo_.begin(), finales_tramo_.end(), tiempo);
  return upper == finales_tramo_.end() ? tramos_.size() - 1U :
         static_cast<std::size_t>(upper - finales_tramo_.begin());
}

double GenTrayPol3Waypoints::get_inicio_tramo(std::size_t indice) const
{
  if (indice >= inicios_tramo_.size()) {
    throw std::out_of_range("indice de tramo Pol3 invalido");
  }
  return inicios_tramo_[indice];
}

void GenTrayPol3Waypoints::comprobar_vector(
  const std::vector<double> & values, const char * name) const
{
  if (values.size() != num_axes_) {
    throw std::invalid_argument(
            std::string("El vector '") + name + "' debe tener tamaño " +
            std::to_string(num_axes_));
  }
}

}  // namespace lib_tray
