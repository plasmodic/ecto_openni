#include <ecto/ecto.hpp>

#include <vector>
#include <sstream>

#include <boost/shared_ptr.hpp>
#include <boost/integer.hpp>

//openni includes
#include <XnCppWrapper.h>
#include "enums.hpp"
#define NI_STATUS_ERROR(x) \
  do{std::stringstream s; s << x << std::string(xnGetStatusString(status)) << std::endl << __LINE__ << ":" << __FILE__ << std::endl;std::cerr << s.str() << std::endl;;}while(false)

using ecto::tendrils;
using ecto::spore;
namespace
{
  std::ostream&
  operator<<(std::ostream& out, const XnMapOutputMode& m)
  {
    out << "xres= " << m.nXRes << " yres= " << m.nYRes << " fps= " << m.nFPS;
    return out;
  }
}

namespace ecto_openni
{
//#define XN_QQVGA_X_RES  160
//#define XN_QQVGA_Y_RES  120
//
//#define XN_CGA_X_RES  320
//#define XN_CGA_Y_RES  200
//
//#define XN_QVGA_X_RES 320
//#define XN_QVGA_Y_RES 240
//
//#define XN_VGA_X_RES  640
//#define XN_VGA_Y_RES  480
//
//#define XN_SVGA_X_RES 800
//#define XN_SVGA_Y_RES 600
//
//#define XN_XGA_X_RES  1024
//#define XN_XGA_Y_RES  768
//
//#define XN_720P_X_RES 1280
//#define XN_720P_Y_RES 720
//
//#define XN_SXGA_X_RES 1280
//#define XN_SXGA_Y_RES 1024
//
//#define XN_UXGA_X_RES 1600
//#define XN_UXGA_Y_RES 1200
//
//#define XN_1080P_X_RES  1920
//#define XN_1080P_Y_RES  1080
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

    template<typename OutputMode>
    void
    setMode(OutputMode& mode, ResolutionMode m)
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

    template<typename Generator>
    void
    enumerate_modes(Generator& g, XnMapOutputMode& desired_mode, bool exact)
    {
      XnMapOutputMode modes[100];
      XnUInt32 n = 100;
      g.GetSupportedMapOutputModes(modes, n);
      XnMapOutputMode best;
      double best_distance = std::numeric_limits<double>::max();
      for (unsigned i = 0; i < n; i++)
      {
        XnMapOutputMode m = modes[i];
        float distance = std::sqrt(
            std::pow(desired_mode.nFPS - m.nFPS, 2) + std::pow(desired_mode.nXRes - m.nXRes, 2)
            + std::pow(desired_mode.nYRes - m.nYRes, 2));
        //std::cout << "Mode " << i << " : " << m << " distance=" << distance << std::endl;
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

    void
    init_ps(bool registration, ResolutionMode m)
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
    init_kinect(ResolutionMode m)
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
    NiStuffs(int index, ResolutionMode rgb_res, ResolutionMode depth_res, int rgb_fps, int depth_fps, bool registration,
             bool synchronize, Device device_type)
    {
      XnStatus status = XN_STATUS_OK;
      depthOutputMode.nFPS = depth_fps;
      imageOutputMode.nFPS = rgb_fps;
      setMode(depthOutputMode, depth_res);
      setMode(imageOutputMode, rgb_res);
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

      if (registration)
        set_depth_registration_on();

      if (synchronize)
        set_sync_on();

      status = context.StartGeneratingAll();
      if (status != XN_STATUS_OK)
        NI_STATUS_ERROR("Failed to start generating.");
    }

    ~NiStuffs()
    {
      context.StopGeneratingAll();
#if XN_VERSION >= (1 * 100000000 + 3 * 1000000 + 2 * 10000)
      context.Release();
#else
      context.Shutdown();
#endif
    }

    void
    set_depth_registration_off()
    {
      XnStatus status = depthGenerator.GetAlternativeViewPointCap().ResetViewPoint();
      if (status != XN_STATUS_OK)
        std::cerr << "Couldn't reset depth view point." << std::endl;
    }

    void
    set_sync_on()
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
    set_sync_off()
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
    set_depth_registration_on()
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
    }

    void
    fillDepth(std::vector<uint16_t>& depth, int& depth_width, int& depth_height)
    {
      depth_width = depthMetaData.XRes();
      depth_height = depthMetaData.YRes();
      const XnDepthPixel* pDepthMap = depthMetaData.Data();
      depth.resize(depth_width * depth_height);
      std::memcpy(reinterpret_cast<char*>(depth.data()), pDepthMap,sizeof(uint16_t)*depth.size());
    }

    void
    fillImageRGB(std::vector<uint8_t>& image, int& image_width, int& image_height, int& nchannels)
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
        grabAll(std::vector<uint8_t>& image, std::vector<uint16_t>& depth, int& image_width, int& image_height,
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
      }
      ;

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
      p.declare<int>("rgb_fps", "The rgb frame rate.", 30);
      p.declare<int>("depth_fps", "The depth frame rate.", 30);
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
            new NiStuffs(*device_number, *rgb_resolution, *depth_resolution, *rgb_fps, *depth_fps, *registration_on,
                         *synchronize, *device));
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
    ecto::spore<int> depth_width, depth_height, image_width, image_height, image_channels, rgb_fps, depth_fps,
        device_number;
    ecto::spore<DepthDataConstPtr> depth_buffer;
    ecto::spore<RgbDataConstPtr> image_buffer;
    ecto::spore<ResolutionMode> rgb_resolution, depth_resolution;
    ecto::spore<bool> registration_on, synchronize;
    ecto::spore<Device> device;

  }
  ;
}

ECTO_CELL(ecto_openni, ecto_openni::Capture, "Capture", "Raw data capture off of an OpenNI device.");
