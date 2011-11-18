/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 2011 Willow Garage, Inc.
 *    Ethan Rublee (erublee@willowgarage.com)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <openni_wrapper/openni_acquire.h>
#include <openni_wrapper/openni_driver.h>

#include <vector>
#include <map>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace boost;
using namespace std;

namespace openni_wrapper
{
  // structure for simplified capture using WaitFor
  template<typename OutputMode>
  void
  NiStuffs::setOutputResolution(OutputMode& mode, ResolutionMode m)
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

  template<typename OutputMode>
  void
  NiStuffs::setOutputFPS(OutputMode& mode, FpsMode m)
  {
    switch (m)
      {
      case FPS_60:
	mode.nFPS = 60;
	break;
      case FPS_30:
	mode.nFPS = 30;
	break;
      case FPS_25:
	mode.nFPS = 25;
	break;
      case FPS_15:
	mode.nFPS = 15;
	break;
      }
  }


  template<typename Generator>
  void
  NiStuffs::enumerate_modes(Generator& g, XnMapOutputMode& desired_mode, bool exact)
  {
    //      std::cout << std::endl << "Desired mode " << desired_mode << std::endl << std::endl;
    XnMapOutputMode modes[100];
    XnUInt32 n = 100;
    g.GetSupportedMapOutputModes(modes, n);
    XnMapOutputMode best;
    double best_distance = 10000;
    for (unsigned i = 0; i < n; i++)
      {
        XnMapOutputMode m = modes[i];
        int distance = abs(int(desired_mode.nFPS - m.nFPS)) + abs((int)(desired_mode.nXRes - m.nXRes)) +
	  abs((int)(desired_mode.nYRes - m.nYRes));
	//        std::cout << "Mode " << i << " : " << m << " distance=" << distance << std::endl;
        if (distance < best_distance)
	  {
	    best_distance = distance;
	    best = m;
	  }
      }

    if (!exact)
      {
	std::cout << "The best mode is : " << best << std::endl;
	desired_mode = best;
      }
  }

  void
  NiStuffs::init_ps(bool registration, ResolutionMode m)
  {

    XnUInt64 inputformat = XN_IO_IMAGE_FORMAT_YUV422;
    if (m == SXGA_RES)
      {
        inputformat = XN_IO_IMAGE_FORMAT_BAYER;
      }

    XnStatus status = imageGenerator.SetIntProperty("InputFormat", inputformat);
    if (status != XN_STATUS_OK)
      NI_STATUS_ERROR("Error setting the image input format. ");

    status = imageGenerator.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);
    if (status != XN_STATUS_OK)
      NI_STATUS_ERROR("Failed to  SetPixelFormat: ");

    if (registration)
      {
        // RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS
        status = depthGenerator.SetIntProperty("RegistrationType", XN_PROCESSING_HARDWARE);
        if (status != XN_STATUS_OK)
	  {
	    NI_STATUS_ERROR("Error setting the registration type. Reason: %s");
	  }
      }

  }

  void
  NiStuffs::init_kinect(ResolutionMode m)
  {
    XnUInt64 inputformat = XN_IO_IMAGE_FORMAT_UNCOMPRESSED_BAYER;
    if (m == SXGA_RES)
      {
        inputformat = XN_IO_IMAGE_FORMAT_UNCOMPRESSED_BAYER;
      }

    XnStatus status = imageGenerator.SetIntProperty("InputFormat", inputformat);
    if (status != XN_STATUS_OK)
      NI_STATUS_ERROR("Error setting the image input format. ");

    status = imageGenerator.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);
    if (status != XN_STATUS_OK)
      NI_STATUS_ERROR("Failed to  SetPixelFormat: ");

    // RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS
    status = depthGenerator.SetIntProperty("RegistrationType", XN_PROCESSING_SOFTWARE);
    if (status != XN_STATUS_OK)
      {
        NI_STATUS_ERROR("Error setting the registration type. Reason: %s");
      }
  }

  //http://en.wikipedia.org/wiki/Resource_Acquisition_Is_Initialization
  NiStuffs::NiStuffs(int index, ResolutionMode rgb_res, ResolutionMode depth_res, 
                     FpsMode rgb_fps, FpsMode depth_fps, bool registration,
		     bool synchronize)
  {
    XnStatus status = XN_STATUS_OK;
    setOutputResolution(depthOutputMode, depth_res);
    setOutputResolution(imageOutputMode, rgb_res);
    setOutputFPS(depthOutputMode, depth_fps);
    setOutputFPS(imageOutputMode, rgb_fps);
    status = context.Init();

    // Initialize and configure the context.
    if (status != XN_STATUS_OK)
      NI_STATUS_ERROR("Fail on init: ");

    // Find devices
    OpenNIDriver& driver = OpenNIDriver::getInstance ();
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
      {
	cout << "Device: " << deviceIdx << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: "
	     << driver.getProductName (deviceIdx) << ", connected: " << (int)driver.getBus (deviceIdx) << " @ "
	     << (int)driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      }
    if (driver.getNumberDevices() == 0)
      {
	cout << "No devices connected." << endl;
	return;
      }

    xn::NodeInfoList devicesList;
    status = context.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, devicesList, 0);
    if (status != XN_STATUS_OK)
      NI_STATUS_ERROR("Failed to enumerate production trees: ");

    // Chose device according to index
    xn::NodeInfoList::Iterator it = devicesList.Begin();
    for (int i = 0; i < index; ++i)
      it++;

    //todo FIXME allow enumeration and selection of devices.
    xn::NodeInfo deviceNode = *it;
    status = context.CreateProductionTree(deviceNode);
    if (status != XN_STATUS_OK)
      NI_STATUS_ERROR("Failed to create production tree: ");

    // Associate generators with context.
    status = depthGenerator.Create(context);
    if (status != XN_STATUS_OK)
      NI_STATUS_ERROR("Failed to create depth generator: ");

    status = imageGenerator.Create(context);
    if (status != XN_STATUS_OK)
      NI_STATUS_ERROR("Failed to create image generator: ");

    //std::cout << "Depth modes:" << std::endl;
    enumerate_modes(depthGenerator, depthOutputMode, true);
    //std::cout << "image modes:" << std::endl;
    enumerate_modes(imageGenerator, imageOutputMode, true);

    // Set map output mode.
    status = depthGenerator.SetMapOutputMode(depthOutputMode);
    if (status != XN_STATUS_OK)
      NI_STATUS_ERROR("Failed to set SetMapOutputMode:\n ");

    status = imageGenerator.SetMapOutputMode(imageOutputMode);
    if (status != XN_STATUS_OK)
      NI_STATUS_ERROR("Failed to set SetMapOutputMode:\n ");

    Device device_type;
    if (strcmp(driver.getProductName (index), "PrimeSense Device")==0)
      device_type = PRIMESENSE;
    if (!strcmp(driver.getProductName (index), "Asus"))
      device_type = ASUS_XTION_PRO_LIVE;
    if (!strcmp(driver.getProductName (index), "Kinect"))
      device_type = KINECT;

    switch (device_type)
      {
      case KINECT:
	init_kinect(rgb_res);
	break;
      case PRIMESENSE:
      case ASUS_XTION_PRO_LIVE:
	init_ps(registration, rgb_res);
	break;
      }

    //      std::cout << "Setting reg and sync" << std::endl;
    if (registration)
      set_depth_registration_on();

    if (synchronize)
      set_sync_on();

    status = context.StartGeneratingAll();
    if (status != XN_STATUS_OK)
      NI_STATUS_ERROR("Failed to start generating.");
  }

  NiStuffs::~NiStuffs()
  {
    context.StopGeneratingAll();
#if XN_VERSION >= (1 * 100000000 + 3 * 1000000 + 2 * 10000)
    context.Release();
#else
    context.Shutdown();
#endif
  }

  void
  NiStuffs::set_depth_registration_off()
  {
    XnStatus status = depthGenerator.GetAlternativeViewPointCap().ResetViewPoint();
    if (status != XN_STATUS_OK)
      std::cerr << "Couldn't reset depth view point." << std::endl;
  }

  void
  NiStuffs::set_sync_on()
  {
    std::cout << " attempting to turn on sync..." << std::endl;
    if (depthGenerator.IsCapabilitySupported(XN_CAPABILITY_FRAME_SYNC))
      {
        if (depthGenerator.GetFrameSyncCap().CanFrameSyncWith(imageGenerator)
            && !depthGenerator.GetFrameSyncCap().IsFrameSyncedWith(imageGenerator))
	  {
	    XnStatus status = depthGenerator.GetFrameSyncCap().FrameSyncWith(imageGenerator);
	    if (status != XN_STATUS_OK)
	      {
		NI_STATUS_ERROR("Failed to start frame sync");
	      }
	  }
        if (depthGenerator.GetFrameSyncCap().IsFrameSyncedWith(imageGenerator))
	  {
	    std::cout << "Successful sync between depth and image." << std::endl;
	  }
        else
	  {
	    std::cerr << "Depth/Image sync could not be enabled." << std::endl;
	  }
      }
    else
      {
        std::cerr << "Depth/Image sync is not supported." << std::endl;
      }
  }

  void
  NiStuffs::set_sync_off()
  {
    if (depthGenerator.IsCapabilitySupported(XN_CAPABILITY_FRAME_SYNC))
      {
        std::cout << " attempting to turn off sync..." << std::endl;
        if (depthGenerator.GetFrameSyncCap().CanFrameSyncWith(imageGenerator)
            && depthGenerator.GetFrameSyncCap().IsFrameSyncedWith(imageGenerator))
	  {
	    XnStatus status = depthGenerator.GetFrameSyncCap().StopFrameSyncWith(imageGenerator);
	    if (status != XN_STATUS_OK)
	      {
		NI_STATUS_ERROR("Failed to stop frame sync");
	      }
	  }
      }
  }

  void
  NiStuffs::set_depth_registration_on()
  {
    std::cout << " attempting to turn on registration..." << std::endl;
    bool set = false;
    if (!depthGenerator.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT))
      {
        std::cerr << "Depth registration is not supported by this device." << std::endl;
        return;
      }
    if (depthGenerator.GetAlternativeViewPointCap().IsViewPointSupported(imageGenerator))
      {
        XnStatus status = depthGenerator.GetAlternativeViewPointCap().SetViewPoint(imageGenerator);
        if (status == XN_STATUS_OK)
	  {
	    set = true;
	  }
        else
	  {
	    NI_STATUS_ERROR("Failed to set view point");
	  }
      }
    if (!set)
      {
        std::cerr << "Could not set depth registration on." << std::endl;
      }
    else
      std::cout << "Successful registration between depth and image." << std::endl;
  }

  void
  NiStuffs::fillDepth(std::vector<uint16_t>& depth, int& depth_width, int& depth_height)
  {
    depth_width = depthMetaData.XRes();
    depth_height = depthMetaData.YRes();
    const XnDepthPixel* pDepthMap = depthMetaData.Data();
    depth.resize(depth_width * depth_height);
    std::memcpy(reinterpret_cast<char*>(depth.data()), pDepthMap,sizeof(uint16_t)*depth.size());
  }

  void
  NiStuffs::fillImageRGB(std::vector<uint8_t>& image, int& image_width, int& image_height, int& nchannels)
  {
    image_width = imageMetaData.FullXRes();
    image_height = imageMetaData.FullYRes();
    //      int bytes_per_pixel = imageMetaData.BytesPerPixel();
    //      int pixel_format = imageMetaData.PixelFormat();
    //      int xoffset = imageMetaData.XOffset();
    //      int yoffset = imageMetaData.YOffset();
    nchannels = imageMetaData.BytesPerPixel();
    image.resize(size_t(imageMetaData.DataSize()));
    memcpy(static_cast<void*>(image.data()), static_cast<const void*>(imageMetaData.Data()), image.size());
  }

  void
  NiStuffs::grabAll(std::vector<uint8_t>& image, std::vector<uint16_t>& depth, int& image_width, int& image_height,
		    int& nchannels, int& depth_width, int& depth_height)
  {
    XnStatus status = context.WaitAndUpdateAll();
    if (status != XN_STATUS_OK)
      NI_STATUS_ERROR("Failed to update all contexts.");
    depthGenerator.GetMetaData(depthMetaData);
    imageGenerator.GetMetaData(imageMetaData);
    fillDepth(depth, depth_width, depth_height);
    fillImageRGB(image, image_width, image_height, nchannels);
  }

} // namespace openni_wrapper
