#ifndef LIB_TRAY__GEN_TRAY_POL3_WAYPOINTS_HPP_
#define LIB_TRAY__GEN_TRAY_POL3_WAYPOINTS_HPP_

#include "lib_tray/gen_tray_pol3.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace lib_tray
{

// Encadena perfiles Pol3 sin exponer derivadas en los waypoints de misión.
class GenTrayPol3Waypoints
{
public:
  explicit GenTrayPol3Waypoints(std::size_t num_axes, double waypoint_blend_sec = 3.0);

  void calcular_trayectoria(
    const std::vector<double> & posiciones_iniciales,
    const std::vector<double> & velocidades_iniciales,
    const std::vector<std::vector<double>> & destinos,
    const std::vector<double> & tiempos_llegada);

  // Variante usada para conservar exactamente los tiempos independientes por
  // eje del Pol3 legacy cuando existe un único destino directo.
  void calcular_trayectoria_por_eje(
    const std::vector<double> & posiciones_iniciales,
    const std::vector<double> & velocidades_iniciales,
    const std::vector<std::vector<double>> & destinos,
    const std::vector<std::vector<double>> & tiempos_llegada_por_eje);

  std::vector<std::array<double, 5>> evaluar(double tiempo) const;
  void evaluar(double tiempo, std::vector<std::array<double, 5>> & salida) const;

  std::size_t get_num_ejes() const;
  double get_tiempo_total() const;

  // Metadatos de lectura para telemetria: no alteran la evaluacion del perfil.
  std::size_t get_num_tramos() const;
  std::size_t get_indice_tramo(double tiempo) const;
  double get_inicio_tramo(std::size_t indice) const;

private:
  void calcular_impl(
    const std::vector<double> & posiciones_iniciales,
    const std::vector<double> & velocidades_iniciales,
    const std::vector<std::vector<double>> & destinos,
    const std::vector<std::vector<double>> & tiempos_llegada_por_eje);

  void comprobar_vector(const std::vector<double> & values, const char * name) const;

  std::size_t num_axes_{0};
  double waypoint_blend_sec_{3.0};
  std::vector<GenTrayPol3> tramos_;
  std::vector<double> inicios_tramo_;
  std::vector<double> finales_tramo_;
  double tiempo_total_{0.0};
};

}  // namespace lib_tray

#endif  // LIB_TRAY__GEN_TRAY_POL3_WAYPOINTS_HPP_
