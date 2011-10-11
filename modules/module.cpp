#include <ecto/ecto.hpp>

#include "enums.hpp"
using namespace ecto_openni;

ECTO_DEFINE_MODULE(ecto_openni){
  boost::python::enum_<ResolutionMode>("ResolutionMode")
    .value("QQVGA_RES",QQVGA_RES)
    .value("CGA_RES",CGA_RES)
    .value("QVGA_RES",QVGA_RES)
    .value("VGA_RES",VGA_RES)
    .value("XGA_RES",XGA_RES)
    .value("HD720P_RES",HD720P_RES)
    .value("SXGA_RES",SXGA_RES)
    .value("UXGA_RES",UXGA_RES)
    .value("HD1080P_RES",HD1080P_RES)
    .export_values()
    ;

  boost::python::enum_<Device>("Device")
    .value("KINECT",KINECT)
    .value("PRIMESENSE",PRIMESENSE)
    .value("ASUS_XTION_PRO_LIVE",ASUS_XTION_PRO_LIVE)
    .export_values()
    ;

  boost::python::enum_<StreamMode>("StreamMode")
    .value("DEPTH",DEPTH)
    .value("RGB",RGB)
    .value("IR",IR)
    .value("DEPTH_RGB",DEPTH_RGB)
    .value("DEPTH_IR",DEPTH_IR)
    .export_values()
    ;
}
