#ifndef LIB_TRAY__GEN_TRAY_VELTRAP_WAYPOINTS_HPP_
#define LIB_TRAY__GEN_TRAY_VELTRAP_WAYPOINTS_HPP_

#include "lib_tray/gen_tray_veltrap.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace lib_tray
{

// Variante por waypoints que conserva que VelTrap calcula sus propios tiempos.
class GenTrayVelTrapWaypoints
{
public:
  explicit GenTrayVelTrapWaypoints(std::size_t num_axes);

  void calcular_trayectoria(
    const std::vector<double> & posiciones_iniciales,
    const std::vector<double> & velocidades_iniciales,
    const std::vector<std::vector<double>> & destinos,
    const std::vector<double> & velocidades_maximas,
    double tiempo_aceleracion);

  std::vector<std::array<double, 5>> evaluar(double tiempo) const;
  void evaluar(double tiempo, std::vector<std::array<double, 5>> & salida) const;

  double get_tiempo_total() const;
  const std::vector<double> & get_tiempos_llegada() const;

private:
  void comprobar_vector(const std::vector<double> & values, const char * name) const;

  std::size_t num_axes_{0};
  std::vector<GenTrayVelTrap> tramos_;
  std::vector<double> inicios_tramo_;
  std::vector<double> tiempos_llegada_;
  double tiempo_total_{0.0};
};

}  // namespace lib_tray

#endif  // LIB_TRAY__GEN_TRAY_VELTRAP_WAYPOINTS_HPP_
