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
    QQVGA_RES, CGA_RES, QVGA_RES, VGA_RES, XGA_RES, HD720P_RES, SXGA_RES, UXGA_RES, HD1080P_RES
  };

  template<typename OutputMode>
  void
  setMode(OutputMode& mode, ResolutionMode m)
  {
    switch (m)
    {
      case QQVGA_RES:
        mode.nXRes = XN_QQVGA_X_RES;
        mode.nYRes = XN_QQVGA_Y_RES;
        break;
      case CGA_RES:
        mode.nXRes = XN_CGA_X_RES;
        mode.nYRes = XN_CGA_Y_RES;
        break;
      case QVGA_RES:
        mode.nXRes = XN_QVGA_X_RES;
        mode.nYRes = XN_QVGA_Y_RES;
        break;
      case VGA_RES:
        mode.nXRes = XN_VGA_X_RES;
        mode.nYRes = XN_VGA_Y_RES;
        break;
      case XGA_RES:
        mode.nXRes = XN_XGA_X_RES;
        mode.nYRes = XN_XGA_Y_RES;
        break;
      case HD720P_RES:
        mode.nXRes = XN_720P_X_RES;
        mode.nYRes = XN_720P_Y_RES;
        break;
      case UXGA_RES:
        mode.nXRes = XN_UXGA_X_RES;
        mode.nYRes = XN_UXGA_Y_RES;
        break;
      case HD1080P_RES:
        mode.nXRes = XN_1080P_X_RES;
        mode.nYRes = XN_1080P_Y_RES;
        break;
      case SXGA_RES:
        mode.nXRes = XN_SXGA_X_RES;
        mode.nYRes = XN_SXGA_Y_RES;
        break;
    }
  }

  enum FpsMode
  {
    FPS_15 = 15, FPS_30 = 30, FPS_60 = 60,
  };

  enum Device
  {
    KINECT, PRIMESENSE, ASUS_XTION_PRO_LIVE,
  };

  enum StreamMode
  {
    IR = 1, DEPTH = 2, RGB = 4, GRAY = 8, DEPTH_RGB = DEPTH | RGB, DEPTH_GRAY = DEPTH | GRAY, DEPTH_IR = DEPTH | IR,
  };
}
