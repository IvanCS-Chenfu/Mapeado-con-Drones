#include "lib_tray/gen_tray_pol3.hpp"

#include <array>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <vector>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Uso:\n";
    std::cerr << "  ros2 run lib_tray test_gen_tray_pol3 <tiempo>\n\n";
    std::cerr << "Ejemplo:\n";
    std::cerr << "  ros2 run lib_tray test_gen_tray_pol3 1.5\n\n";

    return 1;
  }

  double t = 0.0;

  try {
    t = std::stod(argv[1]);
  } catch (const std::exception & e) {
    std::cerr << "Error: el tiempo introducido no es válido.\n";
    std::cerr << "Valor recibido: " << argv[1] << "\n";
    return 1;
  }

  try {
    constexpr std::size_t N_ejes = 3;

    lib_tray::GenTrayPol3 trayectoria(N_ejes);


    const std::vector<double> posiciones_iniciales = {0.0, 1.0, 2.0};
    const std::vector<double> posiciones_finales = {10.0, 5.0, -1.0};
    const std::vector<double> velocidades_iniciales = {0.0, 0.0, 0.0};
    const std::vector<double> velocidades_finales = {0.0, 0.0, 0.0};
    const std::vector<double> tiempos_finales = {5.0, 3.0, 7.0};
    const std::vector<bool> finales_absolutas = {true, true, false};

    trayectoria.calcular_trayectoria(
      posiciones_iniciales, posiciones_finales,
      velocidades_iniciales, velocidades_finales, tiempos_finales,
      finales_absolutas);


    std::vector<std::array<double, 5>> salida;

    const auto t_inicio = std::chrono::steady_clock::now();
    trayectoria.evaluar(t, salida);
    const auto t_fin = std::chrono::steady_clock::now();

    const auto duracion_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(t_fin - t_inicio).count();
    const auto duracion_us =
      std::chrono::duration_cast<std::chrono::microseconds>(t_fin - t_inicio).count();

    std::cout << "\n";
    std::cout << "Test GenTrayPol3\n";
    std::cout << "================\n";
    std::cout << "Tiempo evaluado t = " << t << " s\n";
    std::cout << "Numero de ejes    = " << trayectoria.get_num_ejes() << "\n\n";

    std::cout << "Salida:\n";
    std::cout << "Eje | Posicion | Velocidad | Aceleracion | Jerk | Ratio [%]\n";
    std::cout << "-------------------------------------------------------------\n";

    for (std::size_t i = 0; i < salida.size(); ++i) {
      std::cout << i << " | " << salida[i][0] << " | " << salida[i][1] << " | " << salida[i][2] <<
        " | " << salida[i][3] << " | " << salida[i][4] << "\n";
    }

    std::cout << "\n";
    std::cout << "Tiempo de ejecucion de trayectoria.evaluar():\n";
    std::cout << "  " << duracion_ns << " ns\n";
    std::cout << "  " << duracion_us << " us\n";
    std::cout << "\n";

  } catch (const std::exception & e) {
    std::cerr << "Excepcion capturada:\n";
    std::cerr << "  " << e.what() << "\n";
    return 1;
  }

  return 0;
}
