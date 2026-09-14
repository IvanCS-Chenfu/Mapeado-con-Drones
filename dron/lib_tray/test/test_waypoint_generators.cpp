#include "lib_tray/gen_tray_pol3.hpp"
#include "lib_tray/gen_tray_pol3_waypoints.hpp"
#include "lib_tray/gen_tray_veltrap_waypoints.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace
{

TEST(GenTrayPol3Waypoints, UnDestinoReplicaElPerfilLegacyPorEje)
{
  const std::vector<double> start{1.0, -2.0, 0.5, 0.2};
  const std::vector<double> velocity{0.1, -0.2, 0.0, 0.05};
  const std::vector<double> target{3.0, 1.0, 2.0, 0.6};
  const std::vector<double> times{4.0, 3.0, 5.0, 2.0};
  lib_tray::GenTrayPol3 legacy(4U);
  legacy.calcular_trayectoria(
    start, target, velocity, std::vector<double>(4U, 0.0), times,
    std::vector<bool>(4U, true));
  lib_tray::GenTrayPol3Waypoints waypoints(4U);
  waypoints.calcular_trayectoria_por_eje(start, velocity, {target}, {times});
  for (const double sample : {0.0, 0.5, 2.0, 4.0, 5.0}) {
    const auto expected = legacy.evaluar(sample);
    const auto actual = waypoints.evaluar(sample);
    for (std::size_t axis = 0U; axis < expected.size(); ++axis) {
      for (std::size_t field = 0U; field < 4U; ++field) {
        EXPECT_NEAR(actual[axis][field], expected[axis][field], 1e-12);
      }
    }
  }
}

TEST(GenTrayPol3Waypoints, EmpalmaPoseYVelocidadSinExigirElVerticeGuia)
{
  lib_tray::GenTrayPol3Waypoints waypoints(1U, 1.0);
  waypoints.calcular_trayectoria({0.0}, {0.0}, {{3.0}, {10.0}}, {4.0, 10.0});
  ASSERT_EQ(waypoints.get_num_tramos(), 3U);
  EXPECT_EQ(waypoints.get_indice_tramo(0.0), 0U);
  EXPECT_EQ(waypoints.get_indice_tramo(3.0), 1U);
  EXPECT_EQ(waypoints.get_indice_tramo(5.0), 2U);
  EXPECT_NEAR(waypoints.get_inicio_tramo(1U), 3.0, 1e-12);
  EXPECT_NEAR(waypoints.get_inicio_tramo(2U), 5.0, 1e-12);
  constexpr double epsilon = 1e-6;
  for (const double transition : {3.0, 5.0}) {
    const auto before = waypoints.evaluar(transition - epsilon);
    const auto after = waypoints.evaluar(transition + epsilon);
    EXPECT_NEAR(before[0][0], after[0][0], 2e-5);
    EXPECT_NEAR(before[0][1], after[0][1], 2e-5);
  }
  const auto guide_time = waypoints.evaluar(4.0);
  EXPECT_GT(std::abs(guide_time[0][0] - 3.0), 1e-3);
  const auto final = waypoints.evaluar(10.0);
  EXPECT_NEAR(final[0][0], 10.0, 1e-12);
  EXPECT_NEAR(final[0][1], 0.0, 1e-12);
}

TEST(GenTrayPol3Waypoints, RechazaTiemposQueSolapanEmpalmes)
{
  lib_tray::GenTrayPol3Waypoints waypoints(1U, 1.0);
  EXPECT_THROW(
    waypoints.calcular_trayectoria({0.0}, {0.0}, {{1.0}, {2.0}}, {1.0, 2.0}),
    std::invalid_argument);
}

TEST(GenTrayPol3Waypoints, ConservaTiempoNominalConEmpalmesDeTresSegundos)
{
  lib_tray::GenTrayPol3Waypoints waypoints(1U, 3.0);
  waypoints.calcular_trayectoria(
    {0.0}, {0.0}, {{2.0}, {4.0}, {6.0}}, {8.0, 16.0, 24.0});
  ASSERT_EQ(waypoints.get_num_tramos(), 5U);
  EXPECT_NEAR(waypoints.get_inicio_tramo(1U), 5.0, 1e-12);
  EXPECT_NEAR(waypoints.get_inicio_tramo(2U), 11.0, 1e-12);
  EXPECT_NEAR(waypoints.get_inicio_tramo(3U), 13.0, 1e-12);
  EXPECT_NEAR(waypoints.get_inicio_tramo(4U), 19.0, 1e-12);
  EXPECT_THROW(
    waypoints.calcular_trayectoria(
      {0.0}, {0.0}, {{2.0}, {4.0}, {6.0}}, {8.0, 14.0, 22.0}),
    std::invalid_argument);
}

TEST(GenTrayVelTrapWaypoints, CalculaSusPropiosTiempos)
{
  lib_tray::GenTrayVelTrapWaypoints waypoints(1U);
  waypoints.calcular_trayectoria({0.0}, {0.0}, {{1.0}, {3.0}}, {1.0}, 1.0);
  ASSERT_EQ(waypoints.get_tiempos_llegada().size(), 2U);
  EXPECT_GT(waypoints.get_tiempos_llegada()[0], 0.0);
  EXPECT_GT(waypoints.get_tiempos_llegada()[1], waypoints.get_tiempos_llegada()[0]);
  const auto final = waypoints.evaluar(waypoints.get_tiempo_total());
  EXPECT_NEAR(final[0][0], 3.0, 1e-12);
}

TEST(GenTrayVelTrapWaypoints, AtraviesaVerticesSinParada)
{
  lib_tray::GenTrayVelTrapWaypoints waypoints(1U);
  waypoints.calcular_trayectoria({0.0}, {0.0}, {{2.0}, {5.0}}, {1.0}, 1.0);
  const auto at_vertex = waypoints.evaluar(waypoints.get_tiempos_llegada()[0]);
  EXPECT_NEAR(at_vertex[0][0], 2.0, 1e-9);
  EXPECT_GT(at_vertex[0][1], 0.0);
}

}  // namespace
