#include "lib_tray/gen_tray_veltrap.hpp"
#include "lib_tray/gen_tray_pol3.hpp"
#include "lib_tray/gen_tray_pol3_waypoints.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct Caso
{
  std::string nombre;
  double destino;
};

void escribir_caso(const std::filesystem::path & output_dir, const Caso & caso)
{
  lib_tray::GenTrayVelTrap trayectoria(1U);
  trayectoria.calcular_trayectoria(
    {0.0}, {caso.destino}, {0.0}, {0.0}, {0.8}, 5.0, {true});

  const auto & coef = trayectoria.get_coeficientes().at(0U);
  const double dt = 0.02;
  const std::size_t samples = static_cast<std::size_t>(std::ceil(coef.tf / dt));

  std::ofstream csv(output_dir / (caso.nombre + ".csv"));
  if (!csv) {
    throw std::runtime_error("No se pudo abrir el CSV de salida");
  }

  csv << "t_s,position_m,velocity_mps,acceleration_mps2,jerk_mps3,ratio_percent\n";
  csv << std::setprecision(12);
  for (std::size_t index = 0U; index <= samples; ++index) {
    const double time = std::min(static_cast<double>(index) * dt, coef.tf);
    const auto values = trayectoria.evaluar(time).at(0U);
    csv << time << ',' << values[0] << ',' << values[1] << ',' << values[2] << ','
        << values[3] << ',' << values[4] << '\n';
  }

  std::cout << caso.nombre << ": destino=" << caso.destino
            << " m, tf=" << coef.tf
            << " s, v_peak=" << coef.v_max
            << " m/s, a=" << coef.a
            << " m/s2, triangular=" << (coef.triangular ? "true" : "false")
            << ", estacionario=" << (coef.estacionario ? "true" : "false") << '\n';
}

void escribir_caso_cubico(const std::filesystem::path & output_dir)
{
  lib_tray::GenTrayPol3 trayectoria(1U);
  trayectoria.calcular_trayectoria(
    {0.0}, {10.0}, {0.0}, {0.0}, {5.0}, {true});

  const auto & coef = trayectoria.get_coeficientes().at(0U);
  const double dt = 0.02;
  const std::size_t samples = static_cast<std::size_t>(std::ceil(coef.tf / dt));

  std::ofstream csv(output_dir / "cubica_x.csv");
  if (!csv) {
    throw std::runtime_error("No se pudo abrir el CSV cubico de salida");
  }

  csv << "t_s,position_m,velocity_mps,acceleration_mps2,jerk_mps3,ratio_percent\n";
  csv << std::setprecision(12);
  for (std::size_t index = 0U; index <= samples; ++index) {
    const double time = std::min(static_cast<double>(index) * dt, coef.tf);
    const auto values = trayectoria.evaluar(time).at(0U);
    csv << time << ',' << values[0] << ',' << values[1] << ',' << values[2] << ','
        << values[3] << ',' << values[4] << '\n';
  }

  std::cout << "cubica_x: destino=10 m, tf=" << coef.tf
            << " s, coeficientes=(" << coef.a0 << ", " << coef.a1 << ", "
            << coef.a2 << ", " << coef.a3 << ")\n";
}

void escribir_waypoints_cubicos(
  const std::filesystem::path & output_dir,
  const std::string & file_stem,
  double blend_sec,
  const std::vector<double> & arrival_times)
{
  constexpr double yaw_rad = 1.5707963267948966;
  const std::vector<double> initial_position = {0.0, 0.0, 1.0, yaw_rad};
  const std::vector<double> initial_velocity(4U, 0.0);
  const std::vector<std::vector<double>> targets = {
    {4.0, 0.0, 1.0, yaw_rad},
    {7.0, 3.0, 1.0, yaw_rad},
    {4.0, 7.0, 1.0, yaw_rad},
    {-1.0, 5.0, 1.0, yaw_rad},
    {-3.0, 1.0, 1.0, yaw_rad},
  };
  lib_tray::GenTrayPol3Waypoints trayectoria(4U, blend_sec);
  trayectoria.calcular_trayectoria(
    initial_position, initial_velocity, targets, arrival_times);

  std::ofstream targets_csv(output_dir / (file_stem + "_objetivos.csv"));
  if (!targets_csv) {
    throw std::runtime_error("No se pudo abrir el CSV de objetivos por waypoints");
  }
  targets_csv << "waypoint_index,t_arrival_s,x_m,y_m,z_m,yaw_rad\n";
  targets_csv << std::setprecision(12);
  for (std::size_t index = 0U; index < targets.size(); ++index) {
    targets_csv << index + 1U << ',' << arrival_times[index] << ','
                << targets[index][0] << ',' << targets[index][1] << ','
                << targets[index][2] << ',' << targets[index][3] << '\n';
  }

  std::ofstream csv(output_dir / (file_stem + ".csv"));
  if (!csv) {
    throw std::runtime_error("No se pudo abrir el CSV de waypoints cubicos");
  }
  csv << "t_s,x_m,y_m,z_m,yaw_rad,x_velocity_mps,y_velocity_mps,"
         "x_acceleration_mps2,y_acceleration_mps2\n";
  csv << std::setprecision(12);
  const double dt = 0.02;
  const std::size_t samples = static_cast<std::size_t>(
    std::ceil(trayectoria.get_tiempo_total() / dt));
  for (std::size_t index = 0U; index <= samples; ++index) {
    const double time = std::min(
      static_cast<double>(index) * dt, trayectoria.get_tiempo_total());
    const auto values = trayectoria.evaluar(time);
    csv << time << ','
        << values[0][0] << ',' << values[1][0] << ',' << values[2][0] << ','
        << values[3][0] << ',' << values[0][1] << ',' << values[1][1] << ','
        << values[0][2] << ',' << values[1][2] << '\n';
  }

  std::cout << file_stem << ": destinos=" << targets.size()
            << ", tramos=" << trayectoria.get_num_tramos()
            << ", tf=" << trayectoria.get_tiempo_total()
            << " s, blend=" << blend_sec << " s, z=1 m, yaw=90 deg\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Uso: generar_perfiles_velocidad <directorio_datos>\n";
    return 1;
  }

  try {
    const std::filesystem::path output_dir(argv[1]);
    std::filesystem::create_directories(output_dir);
    escribir_caso(output_dir, {"trapezoidal_x", 10.0});
    escribir_caso(output_dir, {"triangular_x", 1.0});
    escribir_caso_cubico(output_dir);
    escribir_waypoints_cubicos(
      output_dir, "waypoints_cubica", 5.0, {12.0, 24.0, 36.0, 48.0, 60.0});
    escribir_waypoints_cubicos(
      output_dir, "waypoints_cubica_blend_1s", 1.0, {6.0, 12.0, 18.0, 24.0, 30.0});
  } catch (const std::exception & error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
