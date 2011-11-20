#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>

namespace
{
const uint8_t*
create_lut()
{
  static uint8_t ir_gamma[1024];
  for (int i = 0; i < 1024; i++)
  {
    float v = i / 1024.0;
    v = powf(v, 0.45);
    ir_gamma[i] = v * 256;
  }
  return ir_gamma;
}

static const uint8_t * ir_gamma = create_lut();

struct IRGamma
{
  static void
  declare_io(const ecto::tendrils& p, ecto::tendrils& i, ecto::tendrils& o)
  {
    i.declare(&IRGamma::input_, "image", "An 16 bit IR image.").required(true);
    o.declare(&IRGamma::output_, "image", "The gamma corrected 8 bit version of the IR image.");
  }

  int
  process(const ecto::tendrils& i, const ecto::tendrils& o)
  {
    *output_ = cv::Mat(); //reset the output so that the cv mat is reallocated
    if (input_->empty())
      return ecto::OK;

    const cv::Mat irimg = *input_;
    const int ROWS = irimg.rows;
    const int COLS = irimg.cols;
    // convert to cv::Mat and display
    cv::Mat out(ROWS, COLS, CV_8UC1);
    uint8_t * oi = out.ptr<uint8_t>(0);
    const uint16_t *im2 = irimg.ptr<uint16_t>(0);
    for (int i = 0; i < ROWS * COLS; i++)
    {
      oi[i] = (uint8_t) ir_gamma[im2[i]]; // use gamma-corrected
    }
    *output_ = out;
    return ecto::OK;
  }
  ecto::spore<cv::Mat> input_, output_;

};
}
ECTO_CELL(ecto_openni, IRGamma, "IRGamma",
          "Convert the IR image to a gamma corrected 8bit grayscale image.");

