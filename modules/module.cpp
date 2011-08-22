#include <ecto/ecto.hpp>

#include "enums.hpp"
using namespace ecto_openni;

ECTO_DEFINE_MODULE(ecto_openni){
  boost::python::enum_<ResolutionMode>("ResolutionMode")
    .value("QVGA_RES",QVGA_RES)
    .value("VGA_RES",VGA_RES)
    .value("XGA_RES",XGA_RES)
    .value("SXGA_RES",SXGA_RES)
    .export_values()
    ;

}
