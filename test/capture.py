#!/usr/bin/env python
# problem because Capture doesn't produce cv::Mat
import ecto
from ecto_opencv.highgui import imshow, NiConverter, FPSDrawer
from ecto_opencv.imgproc import ConvertTo
from ecto_opencv.cv_bp import CV_8UC1
from ecto_openni import Capture, DEPTH_RGB

capture = Capture(registration=True)
fps = FPSDrawer('fps')
plasm = ecto.Plasm()
plasm.connect(capture['image_buffer'] >> fps[:],
              fps[:] >> imshow('image display', name='image')[:],
              capture['depth_buffer'] >> imshow('depth display', name='depth')[:],
              )

sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute()
