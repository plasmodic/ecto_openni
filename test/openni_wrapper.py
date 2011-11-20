#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow, NiConverter, FPSDrawer
from ecto_opencv.imgproc import ConvertTo
from ecto_opencv.cv_bp import CV_8UC1
from ecto_openni import OpenNICapture, DEPTH_RGB

capture = OpenNICapture(stream_mode=DEPTH_RGB, registration=True)
fps = FPSDrawer('fps')
plasm = ecto.Plasm()
plasm.connect(capture['image'] >> fps[:],
              fps[:] >> imshow('image display', name='image')[:],
              capture['depth'] >> imshow('depth display', name='depth')[:],
              )

sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute()
