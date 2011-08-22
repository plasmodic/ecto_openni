#pragma once
#include <XnCppWrapper.h>

//#define XN_QQVGA_X_RES  160
//#define XN_QQVGA_Y_RES  120
//
//#define XN_CGA_X_RES  320
//#define XN_CGA_Y_RES  200
//
//#define XN_QVGA_X_RES 320
//#define XN_QVGA_Y_RES 240
//
//#define XN_VGA_X_RES  640
//#define XN_VGA_Y_RES  480
//
//#define XN_SVGA_X_RES 800
//#define XN_SVGA_Y_RES 600
//
//#define XN_XGA_X_RES  1024
//#define XN_XGA_Y_RES  768
//
//#define XN_720P_X_RES 1280
//#define XN_720P_Y_RES 720
//
//#define XN_SXGA_X_RES 1280
//#define XN_SXGA_Y_RES 1024
//
//#define XN_UXGA_X_RES 1600
//#define XN_UXGA_Y_RES 1200
//
//#define XN_1080P_X_RES  1920
//#define XN_1080P_Y_RES  1080

namespace ecto_openni
{
  enum ResolutionMode
  {
    QVGA_RES, VGA_RES, XGA_RES, SXGA_RES
  };

  enum FpsMode
  {
    FPS_15,FPS_30,FPS_60,
  };
}
