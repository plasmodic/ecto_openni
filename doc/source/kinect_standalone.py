#!/usr/bin/env python
import ecto

from ecto_openni import Grabber
from ecto_opencv import imgproc, calib, highgui

grabber = Grabber('ni grabber')
verter = highgui.NiConverter('verter')
fps = highgui.FPSDrawer('fps')

plasm = ecto.Plasm()
plasm.connect(grabber[:] >> verter[:],
              verter['image'] >> fps[:],
              fps[:] >> highgui.imshow('image display', name='image', waitKey=10)[:],
              verter['depth'] >> highgui.imshow('depth display', name='depth', waitKey= -1)[:],
              )

if __name__ == '__main__':
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()
