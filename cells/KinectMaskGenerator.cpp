#include <boost/foreach.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ecto/ecto.hpp>

using ecto::tendrils;

// TODO make the mask different depending on VGA/SXGA
cv::Mat_<uint8_t>
kinectMask()
{
  // Magic value for DepthFilter
  unsigned int left = 15, right = 60, top = 50, bottom = 15;

  cv::Mat mask = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);

  cv::Rect roi(left, top, mask.size().width - (right + left), mask.size().height - (top + bottom));
  cv::Mat sub_mask = mask(roi);
  sub_mask = 255;

  return mask;
}

/** Cell that generates the mask of the Kinect of the size of the visual data, with 255 where the depth can be
 * measured, 0 otherwise
 */
struct KinectMaskGenerator
{
  static void
  declare_params(tendrils& params)
  {
    // TODO, add a callback that invalidates the current tendril for the output
    params.declare(&KinectMaskGenerator::path_, "path", "Full path to the mask image", "");
  }

  static void
  declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    out.declare(&KinectMaskGenerator::mask_, "mask", "The resulting mask");
  }

  int
  process(const tendrils& in, const tendrils& out)
  {
    // Don't do anything if we've already submitted a mask
    if (mask_->empty())
    {
      if (!path_->empty())
        // Taken from Ilya's config files most likely
        *mask_ = cv::imread(*path_);
      else
      {
        // If we have no mask, create it by hand
        *mask_ = kinectMask();
      }
    }

    return ecto::OK;
  }

  ecto::spore<cv::Mat> mask_;
  ecto::spore<std::string> path_;
};

ECTO_CELL(ecto_openni, KinectMaskGenerator, "KinectMaskGenerator", "...");
