#pragma once

#include "multidron_gui_lib/types.hpp"

#include <string>

namespace multidron_gui_lib
{

/// Lee la configuración canónica de objetos fiduciales de Fase 4.
/// No modifica el backend ni crea una segunda geometría: es un consumidor del
/// mismo YAML que usa orbslam3_server.
class FiducialConfigLoader
{
public:
  static bool Load(
    const std::string & path,
    FiducialVector * objects,
    std::string * error);
};

}  // namespace multidron_gui_lib
