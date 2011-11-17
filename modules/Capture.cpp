#include <ecto/ecto.hpp>

#include <vector>
#include <sstream>

#include <boost/shared_ptr.hpp>
#include <boost/integer.hpp>

//openni includes
#include <XnCppWrapper.h>
#include "openni_wrapper/openni_acquire.h"

using ecto::tendrils;
using ecto::spore;

using namespace openni_wrapper;

namespace ecto_openni
{

  struct Capture
  {
    typedef std::vector<uint8_t> RgbData;
    typedef std::vector<uint16_t> DepthData;

    typedef boost::shared_ptr<RgbData> RgbDataPtr;
    typedef boost::shared_ptr<DepthData> DepthDataPtr;

    typedef boost::shared_ptr<const RgbData> RgbDataConstPtr;
    typedef boost::shared_ptr<const DepthData> DepthDataConstPtr;

    static void
    declare_params(tendrils& p)
    {

      p.declare<bool>("registration", "Turn registration on.", true);
      p.declare<bool>("synchronize", "Turn sync on.", false);
      p.declare<std::string>("device_uid", "Unique identifier for device to use. fixme", "NA");
      p.declare<ResolutionMode>("rgb_resolution", "Rgb mode.", VGA_RES);
      p.declare<ResolutionMode>("depth_resolution", "Depth mode.", VGA_RES);
      p.declare<FpsMode>("rgb_fps", "The rgb frame rate.", FPS_30);
      p.declare<FpsMode>("depth_fps", "The depth frame rate.", FPS_30);
      p.declare<int>("device_number", "The device number.", 0);
      p.declare<Device>("device", "The type of device", KINECT);
    }

    static void
    declare_io(const tendrils& p, tendrils& i, tendrils& o)
    {
      o.declare<int>("depth_width", "Depth frame width.");
      o.declare<int>("depth_height", "Depth frame height.");
      o.declare<int>("image_width", "Image frame width.");
      o.declare<int>("image_height", "Image frame height.");
      o.declare<int>("image_channels", "Number of image channels.");
      o.declare<DepthDataConstPtr>("depth_buffer");
      o.declare<RgbDataConstPtr>("image_buffer");
    }

    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      depth_height = o["depth_height"];
      depth_width = o["depth_width"];
      image_width = o["image_width"];
      image_height = o["image_height"];
      image_channels = o["image_channels"];
      image_buffer = o["image_buffer"];
      depth_buffer = o["depth_buffer"];
      rgb_resolution = p["rgb_resolution"];
      depth_resolution = p["depth_resolution"];
      registration_on = p["registration"];
      rgb_fps = p["rgb_fps"];
      depth_fps = p["depth_fps"];
      synchronize = p["synchronize"];
      device_number = p["device_number"];
      device = p["device"];
    }

    int
    process(const tendrils&, const tendrils&)
    {
      if (!nistuffs)
      {
        std::cout << "Connecting to device." << std::endl;
        nistuffs.reset(
            new NiStuffs(*device_number, *rgb_resolution, *depth_resolution, 
                         *rgb_fps, *depth_fps, *registration_on, *synchronize));
        std::cout << "Connected to device." << std::endl;
      }
      DepthDataPtr db(new DepthData());
      RgbDataPtr ib(new RgbData());
      *image_buffer = ib;
      *depth_buffer = db;
      nistuffs->grabAll(*ib, *db, *image_width, *image_height, *image_channels, *depth_width, *depth_height);
      return ecto::OK;
    }

    boost::shared_ptr<NiStuffs> nistuffs;
    ecto::spore<int> depth_width, depth_height, image_width, image_height, image_channels, device_number;
    ecto::spore<DepthDataConstPtr> depth_buffer;
    ecto::spore<RgbDataConstPtr> image_buffer;
    ecto::spore<ResolutionMode> rgb_resolution, depth_resolution;
    ecto::spore<FpsMode> rgb_fps, depth_fps;
    ecto::spore<bool> registration_on, synchronize;

    //TODO: do we need the "device" parameter?

    ecto::spore<Device> device;

  }
  ;
}

ECTO_CELL(ecto_openni, ecto_openni::Capture, "Capture", "Raw data capture off of an OpenNI device.");
