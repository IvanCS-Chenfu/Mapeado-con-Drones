#ifndef LIB_TRAY__GEN_TRAY_POL3_HPP_
#define LIB_TRAY__GEN_TRAY_POL3_HPP_

#include <array>
#include <cstddef>
#include <vector>

namespace lib_tray
{

    // Estructura para obtener los coeficientes de cada polinomio cúbico
    struct CoefCubic
    {
        double a0{0.0};
        double a1{0.0};
        double a2{0.0};
        double a3{0.0};

        double tf{0.0};
    };

    class GenTrayPol3
    {
        public:
            // Debido a que el constructor solo tiene un parámetro, el compilador puede convertir un número en "lib_tray::GenTrayPol3(<NUMERO>)"
            explicit GenTrayPol3(std::size_t N_ejes);   

            void calcular_trayectoria(
                const std::vector<double> & posiciones_iniciales,
                const std::vector<double> & posiciones_finales,
                const std::vector<double> & velocidades_iniciales,
                const std::vector<double> & velocidades_finales,
                const std::vector<double> & tiempos_finales,
                const std::vector<bool> & finales_absolutas);


            std::vector<std::array<double, 5>> evaluar(double t) const;

            void evaluar(double t, std::vector<std::array<double, 5>> & salida) const;

            std::size_t get_num_ejes() const;

            const std::vector<CoefCubic> & get_coeficientes() const;

        private:
            // Para una variable, calcula los coeficientes de su polinomio cúbico
            CoefCubic calcular_coeficientes(double q0, double qf, double v0, double vf, double tf) const;

            // Para una variable, utiliza los coeficientes para calcular las variables deseadas
            std::array<double, 5> evaluar_eje(const CoefCubic & c, double t) const;

            void comprobar_tamano_vector(const std::vector<double> & vector, const char * nombre_vector) const;

            void comprobar_tamano_vector(const std::vector<bool> & vector, const char * nombre_vector) const;

            std::size_t N_ejes_{0};
            std::vector<CoefCubic> coeficientes_;
    };

}  // namespace lib_tray

#endif  // LIB_TRAY__GEN_TRAY_POL3_HPP_