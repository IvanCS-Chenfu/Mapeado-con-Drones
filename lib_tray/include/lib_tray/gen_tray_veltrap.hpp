#ifndef LIB_TRAY__GEN_TRAY_VELTRAP_HPP_
#define LIB_TRAY__GEN_TRAY_VELTRAP_HPP_

#include <array>
#include <cstddef>
#include <vector>

namespace lib_tray
{

    // Estructura para obtener los coeficientes/parámetros de cada trayectoria trapezoidal
    struct CoefTrapVel
    {
        double p0{0.0};
        double pf{0.0};

        double v0{0.0};
        double vf{0.0};

        double tf{0.0};

        double a{0.0};
        double s{1.0};       // +1 o -1
        double v_max{0.0};   // pico

        double t1{0.0};
        double tc{0.0};
        double t2{0.0};

        double p1{0.0};
        double p2{0.0};

        bool triangular{false};
        bool estacionario{false};
    };

    class GenTrayVelTrap
    {
        public:
            // Debido a que el constructor solo tiene un parámetro, el compilador puede convertir un número en "lib_tray::GenTrayVelTrap(<NUMERO>)"
            explicit GenTrayVelTrap(std::size_t N_ejes);

            void calcular_trayectoria(
                const std::vector<double> & posiciones_iniciales,
                const std::vector<double> & posiciones_finales,
                const std::vector<double> & velocidades_iniciales,
                const std::vector<double> & velocidades_finales,
                const std::vector<double> & velocidades_maximas,
                double t_a,
                const std::vector<bool> & finales_absolutas);

            std::vector<std::array<double, 5>> evaluar(double t) const;

            void evaluar(double t, std::vector<std::array<double, 5>> & salida) const;

            std::size_t get_num_ejes() const;

            const std::vector<CoefTrapVel> & get_coeficientes() const;

        private:
            // Para una variable, calcula los parámetros de su perfil trapezoidal
            CoefTrapVel calcular_coeficientes(double q0, double qf, double v0, double vf, double vmax, double t_a) const;

            // Para una variable, utiliza los parámetros para calcular las variables deseadas
            std::array<double, 5> evaluar_eje(const CoefTrapVel & c, double t) const;

            void comprobar_tamano_vector(const std::vector<double> & vector, const char * nombre_vector) const;

            void comprobar_tamano_vector(const std::vector<bool> & vector, const char * nombre_vector) const;

            std::size_t N_ejes_{0};
            std::vector<CoefTrapVel> coeficientes_;
    };

}  // namespace lib_tray

#endif  // LIB_TRAY__GEN_TRAY_VELTRAP_HPP_