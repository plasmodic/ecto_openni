/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include <boost/foreach.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ecto/ecto.hpp>

using ecto::tendrils;

struct KinectKGenerator
{
  static void
  declare_params(tendrils& params)
  {
    params.declare(&KinectKGenerator::vga_, "VGA", "true if VGA, false for SXGA", true);
  }

  static void
  declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    out.declare(&KinectKGenerator::K_, "K", "The usual K for the Kinect");
  }

  int
  process(const tendrils& in, const tendrils& out)
  {
    // Build an adjacency matrix of all the boxes
    // Taken from Ilya's config files
    if (*vga_)
      *K_ = (cv::Mat_<float>(3, 3) << 525., 0., 319.5, 0., 525., 239.5, 0., 0., 1.);
    else
      *K_ = (cv::Mat_<float>(3, 3) << 525, 0, 319.75, 0, 492.1875, 224.765625, 0, 0, 1);

    return ecto::OK;
  }

  ecto::spore<cv::Mat> K_;
  ecto::spore<bool> vga_;
};

ECTO_CELL(ecto_openni, KinectKGenerator, "KinectKGenerator", "...");
