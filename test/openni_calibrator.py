#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow, VideoCapture, NiConverter, FPSDrawer
from ecto_opencv.calib import PatternDetector, PatternDrawer, CameraCalibrator, ASYMMETRIC_CIRCLES_GRID
from ecto_opencv.imgproc import cvtColor, Conversion
import sys


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
capture = xtion_highres(device)
#capture = xtion_vga(device)
#capture = kinect_vga(device)
#capture = kinect_highres(device)


verter = NiConverter('verter')
fps = FPSDrawer('fps')

rows = 5
cols = 3
square_size = 0.04 #4 cm
pattern_type = ASYMMETRIC_CIRCLES_GRID
n_obs = 50
calibration_file = "camera_new.yml"

video = ecto.Passthrough()
pattern_display = imshow(name="pattern", waitKey=10)
rgb2gray = cvtColor(flag=Conversion.RGB2GRAY)
circle_detector = PatternDetector(rows=rows, cols=cols,
                                  pattern_type=pattern_type, square_size=square_size)
camera_calibrator = CameraCalibrator(output_file_name=calibration_file, n_obs=n_obs)
circle_drawer = PatternDrawer(rows=rows, cols=cols)

plasm = ecto.Plasm()
plasm.connect(verter['image'] >> fps[:],
              fps[:] >> imshow('image display', name='image')[:],
              verter['depth'] >> imshow('depth display', name='depth')[:],
              )
plasm.connect(capture[:] >> verter[:],
              verter['image'] >> video[:],
              video[:] >> (circle_drawer["input"], camera_calibrator["image"], rgb2gray["input"]),
              rgb2gray["out"] >> circle_detector["input"],
              circle_drawer["out"] >> pattern_display["input"],
              circle_detector[ "ideal", "out", "found"] >> camera_calibrator["ideal", "points", "found"],
              circle_detector["out", "found"] >> circle_drawer["points", "found"],
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Calibrate a camera using a dot pattern.')
