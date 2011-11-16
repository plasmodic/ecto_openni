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

#ifndef OPENNI_ACQUIRE_H
#define OPENNI_ACQUIRE_H

#include <vector>
#include <sstream>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdint.h>

#include <boost/shared_ptr.hpp>
#include <boost/integer.hpp>
#include "boost/program_options.hpp"

//openni includes
#include <openni_wrapper/enums.hpp>


namespace
{
  std::ostream&
  operator<<(std::ostream& out, const XnMapOutputMode& m)
  {
    out << "xres= " << m.nXRes << " yres= " << m.nYRes << " fps= " << m.nFPS;
    return out;
  }
}

#define NI_STATUS_ERROR(x) \
  do{std::stringstream s; s << x << std::string(xnGetStatusString(status)) << std::endl << __LINE__ << ":" << __FILE__ << std::endl;std::cerr << s.str() << std::endl;;}while(false)

namespace openni_wrapper
{
  struct NiStuffs
  {
  public:
    // OpenNI context
    xn::Context context;
    // Data generators with its metadata
    xn::DepthGenerator depthGenerator;
    xn::DepthMetaData depthMetaData;
    XnMapOutputMode depthOutputMode;

    xn::ImageGenerator imageGenerator;
    xn::ImageMetaData imageMetaData;
    XnMapOutputMode imageOutputMode;

    // Cameras settings:
    // TODO find in OpenNI function to convert z->disparity and remove fields "baseline" and depthFocalLength_VGA
    // Distance between IR projector and IR camera (in meters)
    XnDouble baseline;
    // Focal length for the IR camera in VGA resolution (in pixels)
    XnUInt64 depthFocalLength_VGA;

    // The value for shadow (occluded pixels)
    XnUInt64 shadowValue;
    // The value for pixels without a valid disparity measurement
    XnUInt64 noSampleValue;

    template<typename OutputMode>
    void
    setOutputResolution(OutputMode& mode, ResolutionMode m);

    template<typename OutputMode>
    void
    setOutputFPS(OutputMode& mode, FpsMode m);

    template<typename Generator>
    void
    enumerate_modes(Generator& g, XnMapOutputMode& desired_mode, bool exact);

    typedef enum
    {
      XN_IO_IMAGE_FORMAT_BAYER = 0,
      XN_IO_IMAGE_FORMAT_YUV422 = 1,
      XN_IO_IMAGE_FORMAT_JPEG = 2,
      XN_IO_IMAGE_FORMAT_JPEG_420 = 3,
      XN_IO_IMAGE_FORMAT_JPEG_MONO = 4,
      XN_IO_IMAGE_FORMAT_UNCOMPRESSED_YUV422 = 5,
      XN_IO_IMAGE_FORMAT_UNCOMPRESSED_BAYER = 6,
    } XnIOImageFormats;

    typedef enum XnProcessingType
    {
      XN_PROCESSING_DONT_CARE = 0, XN_PROCESSING_HARDWARE = 1, XN_PROCESSING_SOFTWARE = 2,
    } XnProcessingType;

    void init_ps(bool registration, ResolutionMode m);
    void init_kinect(ResolutionMode m);

    //http://en.wikipedia.org/wiki/Resource_Acquisition_Is_Initialization
    NiStuffs(int index, ResolutionMode rgb_res, ResolutionMode depth_res, FpsMode rgb_fps, FpsMode depth_fps, 
	     bool registration, bool synchronize, Device device_type);

    ~NiStuffs();

  void set_depth_registration_off();
  void set_sync_on();
  void set_sync_off();
  void set_depth_registration_on();
  void fillDepth(std::vector<uint16_t>& depth, int& depth_width, int& depth_height);
  void fillImageRGB(std::vector<uint8_t>& image, int& image_width, int& image_height, int& nchannels);
  void grabAll(std::vector<uint8_t>& image, std::vector<uint16_t>& depth, int& image_width, int& image_height,
	       int& nchannels, int& depth_width, int& depth_height);

  };

} // namespace openni_wrapper

#endif // OPENNI_ACQUIRE_H
