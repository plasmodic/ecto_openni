#!/usr/bin/env python
import ecto

from ecto_opencv import imgproc, calib, highgui

def kinect_highres(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.SXGA_RES,
                   depth_resolution=ResolutionMode.VGA_RES,
                   rgb_fps=15, depth_fps=30,
                   device_number=device_n,
                   registration=True,
                   synchronize=False,
                   device=Device.KINECT
                   )

def kinect_vga(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.VGA_RES,
                   depth_resolution=ResolutionMode.VGA_RES,
                   rgb_fps=30, depth_fps=30,
                   device_number=device_n,
                   registration=True,
                   synchronize=False,
                   device=Device.KINECT
                   )
def xtion_highres(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.SXGA_RES,
                   depth_resolution=ResolutionMode.VGA_RES,
                   rgb_fps=30, depth_fps=30,
                   device_number=device_n,
                   registration=True,
                   synchronize=True,
                   device=Device.ASUS_XTION_PRO_LIVE
                   )

def xtion_vga(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.VGA_RES,
                   depth_resolution=ResolutionMode.VGA_RES,
                   rgb_fps=30, depth_fps=30,
                   device_number=device_n,
                   registration=True,
                   synchronize=True,
                   device=Device.ASUS_XTION_PRO_LIVE
                   )

device = 0
#capture = xtion_highres(device)
capture = xtion_vga(device)
#capture = kinect_vga(device)
#capture = kinect_highres(device)


verter = highgui.NiConverter('verter')
fps = highgui.FPSDrawer('fps')

plasm = ecto.Plasm()
plasm.connect(capture[:] >> verter[:],
              verter['image'] >> fps[:],
              fps[:] >> highgui.imshow('image display', name='image')[:],
              verter['depth'] >> highgui.imshow('depth display', name='depth')[:],
              )

if __name__ == '__main__':
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()
