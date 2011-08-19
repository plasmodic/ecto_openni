#include <ecto/ecto.hpp>

#include <vector>
#include <sstream>

#include <boost/shared_ptr.hpp>
#include <boost/integer.hpp>

//openni includes
#include <XnCppWrapper.h>

#define NI_STATUS_ERROR(x) \
  do{std::stringstream s; s << x << std::string(xnGetStatusString(status)) << std::endl << __LINE__ << ":" << __FILE__ << std::endl; throw std::runtime_error(s.str());}while(false)

using ecto::tendrils;
using ecto::spore;

namespace ecto_openni
{
  /**
   * OpenNI struct for ease of cell implementation.
   */
  struct NiStuffs
  {
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

    //http://en.wikipedia.org/wiki/Resource_Acquisition_Is_Initialization
    NiStuffs(int index)
    {
      XnStatus status = XN_STATUS_OK;

      // Initialize image output modes (VGA_30HZ by default).
      depthOutputMode.nXRes = imageOutputMode.nXRes = XN_VGA_X_RES; //TODO FIXME turn this into a parameter.
      depthOutputMode.nYRes = imageOutputMode.nYRes = XN_VGA_Y_RES; //TODO FIXME turn this into a parameter.
      depthOutputMode.nFPS = imageOutputMode.nFPS = 30; //TODO FIXME turn this into a parameter.

      status = context.Init();
      // Initialize and configure the context.
      if (status != XN_STATUS_OK)
        NI_STATUS_ERROR("Fail on init: ");

      // Find devices
      xn::NodeInfoList devicesList;
      status = context.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, devicesList, 0);
      if (status != XN_STATUS_OK)
        NI_STATUS_ERROR("Failed to enumerate production trees: ");

      // Chose device according to index
      xn::NodeInfoList::Iterator it = devicesList.Begin();
      for (int i = 0; i < index; ++i)
        it++;

      xn::NodeInfo deviceNode = *it;
      status = context.CreateProductionTree(deviceNode);
      if (status != XN_STATUS_OK)
        NI_STATUS_ERROR("Failed to create production tree: ");

      // Associate generators with context.
      status = depthGenerator.Create(context);
      if (status != XN_STATUS_OK)
        NI_STATUS_ERROR("Failed to create depth generator: ");

      imageGenerator.Create(context);
      if (status != XN_STATUS_OK)
        NI_STATUS_ERROR("Failed to create image generator: ");

      // Set map output mode.
      status = depthGenerator.SetMapOutputMode(depthOutputMode);
      if (status != XN_STATUS_OK)
        NI_STATUS_ERROR("Failed to depth SetMapOutputMode: ");
      // xn::DepthGenerator supports VGA only! (Jan 2011)
      status = imageGenerator.SetMapOutputMode(imageOutputMode);
      if (status != XN_STATUS_OK)
        NI_STATUS_ERROR("Failed to image SetMapOutputMode: ");

      //  Start generating data.
      status = context.StartGeneratingAll();
      if (status != XN_STATUS_OK)
        NI_STATUS_ERROR("Failed to start generating all");
    }

    ~NiStuffs()
    {
      context.StopGeneratingAll();
      context.Shutdown();
    }

    void
    set_depth_registration_off()
    {
      XnStatus status = depthGenerator.GetAlternativeViewPointCap().ResetViewPoint();
      if (status != XN_STATUS_OK)
        NI_STATUS_ERROR("Couldn't reset depth view point: ");
    }

    void
    set_depth_registration_on()
    {
      bool set = false;
      if (!depthGenerator.GetAlternativeViewPointCap().IsViewPointAs(imageGenerator))
      {
        if (depthGenerator.GetAlternativeViewPointCap().IsViewPointSupported(imageGenerator))
        {
          XnStatus status = depthGenerator.GetAlternativeViewPointCap().SetViewPoint(imageGenerator);
          if (status != XN_STATUS_OK)
            NI_STATUS_ERROR("Could not set image as a view point for depth.");

          set = true;
        }
      }
      if (!set)
      {
        std::cerr << "Depth registration is not supported by this device." << std::endl;
      }
    }

    void
    fillDepth(std::vector<uint16_t>& depth, int& depth_width, int& depth_height)
    {
      depth_width = depthMetaData.XRes();
      depth_height = depthMetaData.YRes();
      const XnDepthPixel* pDepthMap = depthMetaData.Data();
      depth.resize(depth_width * depth_height * sizeof(*pDepthMap));
      std::memcpy((char*) (depth.data()), pDepthMap, depth.size());
    }

    void
    fillImageRGB(std::vector<uint8_t>& image, int& image_width, int& image_height, int& nchannels)
    {
      image_width = imageMetaData.XRes();
      image_height = imageMetaData.YRes();
      nchannels = 3;
      const XnRGB24Pixel* pRgbImage = imageMetaData.RGB24Data();

      image.resize(image_height * image_width * nchannels);
      memcpy(image.data(), pRgbImage, image.size());
    }

    void
    grabAll(std::vector<uint8_t>& image, std::vector<uint16_t>& depth, int& image_width, int& image_height,
        int& nchannels, int& depth_width, int& depth_height)
    {
      XnStatus status = context.WaitAndUpdateAll();
      if (status != XN_STATUS_OK)
      throw std::runtime_error("Could not update all openni contexts.");
      depthGenerator.GetMetaData(depthMetaData);
      imageGenerator.GetMetaData(imageMetaData);
      fillDepth(depth, depth_width, depth_height);
      fillImageRGB(image, image_width, image_height, nchannels);
    }
  };

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
      nistuffs.reset(new NiStuffs(0));
      nistuffs->set_depth_registration_on(); //TODO FIXME turn this into a parameter.
    }

    int
    process(const tendrils&, const tendrils&)
    {
      DepthDataPtr db(new DepthData());
      RgbDataPtr ib(new RgbData());
      *image_buffer = ib;
      *depth_buffer = db;
      nistuffs->grabAll(*ib, *db, *image_width, *image_height, *image_channels, *depth_width, *depth_height);
      return ecto::OK;
    }

    boost::shared_ptr<NiStuffs> nistuffs;
    ecto::spore<int> depth_width, depth_height, image_width, image_height, image_channels;
    ecto::spore<DepthDataConstPtr> depth_buffer;
    ecto::spore<RgbDataConstPtr> image_buffer;

  };
}

ECTO_CELL(ecto_openni, ecto_openni::Capture, "Capture", "Raw data capture off of an OpenNI device.");
