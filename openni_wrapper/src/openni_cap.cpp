/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 2011 Willow Garage, Inc.
 *    Kurt Konolige <konolige@willowgarage.com>
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


//
// Simple capture program illustrating use of the openni wrapper calls
//

#include <vector>
#include <sstream>
#include <cmath>
#include <cstring>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/integer.hpp>
#include "boost/program_options.hpp"

#include <opencv2/highgui/highgui.hpp>

//openni includes
#include <openni_wrapper/openni_acquire.h>

using namespace openni_wrapper;

typedef std::vector<uint8_t> RgbData;
typedef std::vector<uint16_t> DepthData;

typedef boost::shared_ptr<RgbData> RgbDataPtr;
typedef boost::shared_ptr<DepthData> DepthDataPtr;

typedef boost::shared_ptr<const RgbData> RgbDataConstPtr;
typedef boost::shared_ptr<const DepthData> DepthDataConstPtr;

boost::shared_ptr<NiStuffs> nistuffs;
int device_number = 0;
ResolutionMode rgb_resolution = VGA_RES, depth_resolution = VGA_RES;
FpsMode rgb_fps = FPS_30, depth_fps = FPS_30;
bool registration_on = true, sync_on = false;
  
int depth_width, depth_height, image_width, image_height, image_channels;
DepthDataConstPtr depth_buffer;
RgbDataConstPtr image_buffer;
//Device device = ASUS_XTION_PRO_LIVE;
Device device = PRIMESENSE;

void printUsage(const char* progName, const boost::program_options::options_description& desc)
{
  std::cout << "\n\n" << progName << " [options]"
	    << desc << std::endl << std::endl;
}

int main(int argc, char **argv)
{
  boost::program_options::options_description generic("====Generic");
  generic.add_options()
    ("help", "show this help and exit")
    ;
  boost::program_options::options_description capture_opts("====Capture");
  capture_opts.add_options()
    ("registration", boost::program_options::bool_switch(&registration_on), "Turn on registration")
    ("synchronization", boost::program_options::bool_switch(&sync_on), "Turn on synchronization")
    ("device number", boost::program_options::value<int>(&device_number)->default_value(0), "Device number")
    ;
  boost::program_options::options_description visible_opts("Options");
  visible_opts.add(generic).add(capture_opts);
  boost::program_options::options_description cmd_line_opts("");
  cmd_line_opts.add(visible_opts);

  boost::program_options::variables_map opt_map;
  try
    {
      boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
				    .options(cmd_line_opts).run(), opt_map);
      boost::program_options::notify(opt_map);
    } catch(boost::program_options::error& e)
    {
      std::cout << "\nERROR: " << e.what() << std::endl;
      printUsage(argv[0], visible_opts);
      return -1;
    }

  // Check
  if(opt_map.count("help"))
    {
      printUsage(argv[0], visible_opts);
      return 0;
    }

  if (!nistuffs)
    {
      std::cout << "Connecting to device." << std::endl;
      nistuffs.reset(new NiStuffs(device_number, rgb_resolution, depth_resolution, rgb_fps, depth_fps, registration_on,
				  sync_on, device));
      std::cout << "Connected to device." << std::endl;
    }

  DepthDataPtr db(new DepthData());
  RgbDataPtr ib(new RgbData());
  image_buffer = ib;
  depth_buffer = db;

  int n = 0;
  cv::namedWindow("image", CV_WINDOW_KEEPRATIO);

  while (1)
    {
      // grab an image
      nistuffs->grabAll(*ib, *db, image_width, image_height, image_channels, depth_width, depth_height);

      // show it
      cv::Mat image(image_height, image_width, CV_8UC3,(void *)image_buffer->data());
      cv::imshow("image", image);
      cv::waitKey(10);

      if ((n % 100) == 0)
	std::cout << "got " << n << std::endl;
      n++;
    }
}

