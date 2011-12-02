#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow, NiConverter, FPSDrawer
from ecto_opencv.imgproc import ConvertTo
from ecto_opencv.cv_bp import CV_8UC1
from ecto_openni import OpenNICapture, DEPTH_RGB
import argparse
from ecto.opts import cell_options

parser = argparse.ArgumentParser(description='My awesome program thing.')

#add our cells to the parser
camera_factory = cell_options(parser, OpenNICapture)
args = parser.parse_args()
print args

#use the factories in conjunction with the parsed arguments, to create our cells.
capture = camera_factory(args)
fps = FPSDrawer('fps')
plasm = ecto.Plasm()
plasm.connect(capture['image'] >> fps[:],
              fps[:] >> imshow('image display', name='image')[:],
              capture['depth'] >> imshow('depth display', name='depth')[:],
              )

sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute()
