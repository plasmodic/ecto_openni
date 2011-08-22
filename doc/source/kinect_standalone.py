#!/usr/bin/env python
import ecto

from ecto_openni import Capture, ResolutionMode
from ecto_opencv import imgproc, calib, highgui

capture = Capture('ni device', rgb_resolution=ResolutionMode.SXGA_RES)
verter = highgui.NiConverter('verter')
fps = highgui.FPSDrawer('fps')

plasm = ecto.Plasm()
plasm.connect(capture[:] >> verter[:],
              verter['image'] >> fps[:],
              fps[:] >> highgui.imshow('image display', name='image', waitKey=10)[:],
              verter['depth'] >> highgui.imshow('depth display', name='depth', waitKey= 10)[:],
              )

if __name__ == '__main__':
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()
