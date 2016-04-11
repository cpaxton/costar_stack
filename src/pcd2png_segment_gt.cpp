/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

/** \brief PCD 2 PNG converter
 *
 * This converter takes 4 inputs: names of the input PCD and Ground Truth PCD files, the output PNG and PNG Ground Truth files, and the name of the field.
 *
 * \author Sergey Alexandrov
 * \author Andrew Hundt <ATHundt@gmail.com>
 *
 */

#include <boost/lexical_cast.hpp>
#include <boost/filesystem/convenience.hpp>

#include <pcl/pcl_config.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/conversions.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


#include <pcl/pcl_macros.h>

#if PCL_VERSION_COMPARE(<, 1, 8, 0)
#include <pcl/io/point_cloud_image_extractors.h>
namespace pcl
{

  struct RGB;

  PCL_EXPORTS RGB
  getRandomColor (double min = 0.2, double max = 2.8);

  /** Color lookup table consisting of 256 colors structured in a maximally
    * discontinuous manner. Generated using the method of Glasbey et al.
    * (see https://github.com/taketwo/glasbey) */
  class PCL_EXPORTS GlasbeyLUT
  {

    public:

      /** Get a color from the lookup table with a given id.
        *
        * The id should be less than the size of the LUT (see size()). */
      static RGB at (unsigned int color_id);

      /** Get the number of colors in the lookup table.
        *
        * Note: the number of colors is different from the number of elements
        * in the lookup table (each color is defined by three bytes). */
      static size_t size ();

      /** Get a raw pointer to the lookup table. */
      static const unsigned char* data ();

  };

}


#if 0
/// Lookup table
static const unsigned char GLASBEY_LUT[] =
{
  77 , 175, 74 ,
  228, 26 , 28 ,
  55 , 126, 184,
  152, 78 , 163,
  255, 127, 0  ,
  255, 255, 51 ,
  166, 86 , 40 ,
  247, 129, 191,
  153, 153, 153,
  0  , 0  , 255,
  255, 0  , 255,
  0  , 255, 248,
  0  , 255, 0  ,
  0  , 101, 255,
  255, 255, 180,
  52 , 68 , 1  ,
  0  , 0  , 68 ,
  96 , 0  , 41 ,
  158, 147, 0  ,
  116, 0  , 185,
  255, 0  , 114,
  0  , 149, 125,
  209, 186, 255,
  255, 183, 156,
  240, 0  , 174,
  208, 255, 245,
  0  , 255, 176,
  170, 255, 93 ,
  0  , 207, 255,
  255, 190, 1  ,
  241, 117, 255,
  52 , 74 , 167,
  150, 166, 103,
  255, 114, 114,
  171, 100, 109,
  161, 0  , 41 ,
  160, 135, 255,
  105, 86 , 121,
  178, 21 , 105,
  0  , 3  , 123,
  255, 221, 236,
  160, 238, 173,
  237, 161, 77 ,
  0  , 141, 255,
  0  , 97 , 109,
  1  , 238, 98 ,
  81 , 0  , 78 ,
  128, 103, 66 ,
  0  , 108, 44 ,
  209, 224, 94 ,
  155, 0  , 255,
  0  , 45 , 223,
  88 , 28 , 0  ,
  166, 2  , 162,
  165, 205, 239,
  84 , 121, 0  ,
  76 , 109, 80 ,
  122, 180, 0  ,
  104, 204, 204,
  145, 95 , 255,
  214, 208, 177,
  185, 130, 176,
  130, 120, 194,
  128, 96 , 0  ,
  247, 161, 255,
  10 , 65 , 119,
  232, 102, 54 ,
  7  , 191, 131,
  105, 54 , 171,
  10 , 177, 0  ,
  215, 191, 105,
  198, 66 , 249,
  140, 255, 145,
  135, 60 , 105,
  254, 170, 191,
  130, 173, 255,
  161, 35 , 0  ,
  197, 255, 0  ,
  40 , 153, 180,
  215, 83 , 185,
  255, 77 , 161,
  128, 175, 147,
  216, 91 , 124,
  193, 144, 91 ,
  210, 196, 0  ,
  232, 39 , 73 ,
  76 , 52 , 116,
  159, 206, 110,
  138, 147, 187,
  140, 5  , 114,
  0  , 56 , 183,
  191, 105, 0  ,
  83 , 58 , 0  ,
  94 , 224, 0  ,
  121, 99 , 99 ,
  212, 123, 104,
  89 , 160, 99 ,
  146, 58 , 54 ,
  231, 46 , 215,
  93 , 245, 200,
  191, 147, 133,
  255, 211, 89 ,
  171, 77 , 214,
  0  , 121, 0  ,
  60 , 14 , 107,
  255, 82 , 1  ,
  112, 115, 43 ,
  0  , 172, 245,
  255, 184, 240,
  1  , 210, 111,
  203, 151, 0  ,
  95 , 114, 129,
  183, 215, 17 ,
  80 , 110, 231,
  201, 25 , 87 ,
  218, 250, 203,
  255, 148, 103,
  255, 217, 163,
  198, 172, 199,
  78 , 139, 135,
  197, 255, 134,
  38 , 0  , 165,
  197, 208, 211,
  193, 117, 225,
  111, 0  , 128,
  147, 255, 238,
  125, 62 , 254,
  112, 213, 78 ,
  198, 76 , 61 ,
  155, 48 , 82 ,
  0  , 199, 176,
  118, 6  , 0  ,
  2  , 106, 192,
  140, 167, 49 ,
  189, 81 , 145,
  254, 162, 38 ,
  134, 138, 106,
  0  , 68 , 17 ,
  122, 73 , 61 ,
  255, 251, 239,
  127, 94 , 193,
  181, 140, 48 ,
  66 , 235, 255,
  189, 140, 218,
  190, 0  , 138,
  132, 177, 185,
  90 , 54 , 202,
  0  , 35 , 131,
  251, 139, 149,
  74 , 0  , 225,
  0  , 106, 90 ,
  106, 199, 155,
  104, 169, 217,
  255, 239, 134,
  44 , 94 , 130,
  126, 0  , 78 ,
  196, 214, 145,
  160, 238, 255,
  222, 215, 255,
  255, 87 , 126,
  170, 161, 255,
  81 , 120, 60 ,
  255, 242, 91 ,
  168, 130, 145,
  158, 153, 64 ,
  211, 123, 156,
  255, 0  , 3  ,
  210, 118, 197,
  0  , 41 , 111,
  198, 178, 125,
  211, 255, 169,
  109, 215, 130,
  41 , 90 , 0  ,
  235, 193, 183,
  114, 58 , 0  ,
  140, 96 , 155,
  163, 223, 193,
  255, 142, 63 ,
  66 , 155, 1  ,
  83 , 96 , 152,
  106, 133, 230,
  255, 85 , 72 ,
  141, 216, 0  ,
  162, 102, 73 ,
  79 , 0  , 146,
  190, 0  , 30 ,
  165, 0  , 193,
  81 , 84 , 255,
  0  , 148, 74 ,
  203, 0  , 255,
  121, 54 , 71 ,
  215, 255, 97 ,
  163, 178, 0  ,
  111, 154, 68 ,
  120, 93 , 222,
  254, 187, 126,
  112, 0  , 27 ,
  204, 59 , 0  ,
  0  , 165, 167,
  151, 255, 0  ,
  104, 41 , 124,
  138, 89 , 113,
  255, 94 , 224,
  86 , 91 , 48 ,
  75 , 255, 76 ,
  204, 190, 67 ,
  255, 224, 0  ,
  49 , 126, 85 ,
  145, 196, 135,
  187, 64 , 79 ,
  255, 130, 233,
  205, 127, 68 ,
  105, 195, 223,
  161, 213, 81 ,
  165, 183, 240,
  125, 255, 180,
  126, 255, 111,
  67 , 255, 145,
  154, 138, 83 ,
  46 , 145, 231,
  118, 121, 0  ,
  154, 2  , 222,
  185, 119, 255,
  255, 0  , 62 ,
  131, 28 , 170,
  177, 70 , 183,
  217, 0  , 115,
  186, 196, 95 ,
  97 , 46 , 97 ,
  84 , 134, 167,
  81 , 54 , 145,
  107, 117, 107,
  51 , 15 , 80 ,
  215, 139, 143,
  255, 247, 203,
  255, 86 , 192,
  153, 91 , 0  ,
  255, 1  , 156,
  183, 79 , 19 ,
  235, 255, 146,
  211, 1  , 224,
  178, 167, 144,
  217, 97 , 0  ,
  134, 91 , 38 ,
  175, 151, 206,
  0  , 182, 63 ,
  210, 40 , 179,
  2  , 213, 42 ,
  94 , 84 , 160,
  136, 48 , 0  ,
  255, 110, 163,
  144, 121, 157,
  153, 61 , 225,
  237, 87 , 255,
  87 , 24 , 206,
  117, 143, 207,
};

/// Number of colors in Glasbey lookup table
static const std::size_t GLASBEY_LUT_SIZE = sizeof (GLASBEY_LUT) / (sizeof (GLASBEY_LUT[0]) * 3);

#endif 

pcl::RGB
pcl::GlasbeyLUT::at (unsigned int color_id)
{
  assert (color_id < GLASBEY_LUT_SIZE);
  pcl::RGB color;
  color.r = GLASBEY_LUT[color_id * 3 + 0];
  color.g = GLASBEY_LUT[color_id * 3 + 1];
  color.b = GLASBEY_LUT[color_id * 3 + 2];
  return (color);
}

size_t
pcl::GlasbeyLUT::size ()
{
  return GLASBEY_LUT_SIZE;
}

const unsigned char*
pcl::GlasbeyLUT::data ()
{
  return GLASBEY_LUT;
}

pcl::RGB
pcl::getRandomColor (double min, double max)
{
  double sum;
  static unsigned stepRGBA = 100;
  double r, g, b;
  do
  {
    sum = 0;
    r = (rand () % stepRGBA) / static_cast<double> (stepRGBA);
    while ((g = (rand () % stepRGBA) / static_cast<double> (stepRGBA)) == r) {}
    while (((b = (rand () % stepRGBA) / static_cast<double> (stepRGBA)) == r) && (b == g)) {}
    sum = r + g + b;
  }
  while (sum <= min || sum >= max);
  pcl::RGB color;
  color.r = uint8_t (r * 255.0);
  color.g = uint8_t (g * 255.0);
  color.b = uint8_t (b * 255.0);
  return (color);
}

#endif // PCL_VERSION_COMPARE(<, 1, 8, 0)

//////////////////////////////////////////////////////////////////////////////////////
/** \brief Image Extractor which uses the data present in the "rgb" or "rgba" fields
  * to produce a color image with rgb8 encoding.
  * \author Sergey Alexandrov
  * \ingroup io
  */

//////////////////////////////////////////////////////////////////////////////////////
/** \brief Image Extractor which uses the data present in the "label" field to produce
  * either monochrome or RGB image where different labels correspond to different
  * colors.
  * See the documentation for ColorMode to learn about available coloring options.
  * \author Sergey Alexandrov
  * \ingroup io
  */
template <typename PointT>
class PointCloudImageExtractorGTLabelFromRGBField : public PointCloudImageExtractor<PointT>
{
  typedef typename PointCloudImageExtractor<PointT>::PointCloud PointCloud;

  public:
    typedef boost::shared_ptr<PointCloudImageExtractorFromLabelField<PointT> > Ptr;
    typedef boost::shared_ptr<const PointCloudImageExtractorFromLabelField<PointT> > ConstPtr;

    /** \brief Different modes for color mapping. */
    enum ColorMode
    {
      /// Shades of gray (according to label id)
      /// \note Labels using more than 16 bits will cause problems
      COLORS_MONO,
      /// Randomly generated RGB colors
      COLORS_RGB_RANDOM,
      /// Fixed RGB colors from the [Glasbey lookup table](http://fiji.sc/Glasbey),
      /// assigned in the ascending order of label id
      COLORS_RGB_GLASBEY,
    };

    /** \brief Constructor. */
    PointCloudImageExtractorGTLabelFromRGBField (int foreground_label =1, int background_label = 0, const ColorMode color_mode = COLORS_MONO)
      : foreground_label_(foreground_label),background_label_(background_label),color_mode_ (color_mode)
    {
    }

    /** \brief Destructor. */
    virtual ~PointCloudImageExtractorGTLabelFromRGBField () {}

    /** \brief Set color mapping mode. */
    inline void
    setColorMode (const ColorMode color_mode)
    {
      color_mode_ = color_mode;
    }

  protected:

    virtual bool
    extractImpl (const PointCloud& cloud, pcl::PCLImage& img) const;

    // Members derived from the base class
    using PointCloudImageExtractor<PointT>::paint_nans_with_black_;

  private:
    
    int foreground_label_;
    int background_label_;
    ColorMode color_mode_;
    
};

// ///////////////////////////////////////////////////////////////////////////////////////////
// template <typename PointT> bool
// pcl::io::PointCloudImageExtractorGTLabelFromRGBField<PointT>::extractImpl (const PointCloud& cloud, pcl::PCLImage& img) const
// {
//   std::vector<pcl::PCLPointField> fields;
//   int field_idx = pcl::getFieldIndex (cloud, "rgb", fields);
//   if (field_idx == -1)
//   {
//     field_idx = pcl::getFieldIndex (cloud, "rgba", fields);
//     if (field_idx == -1)
//       return (false);
//   }
//
//   int field_z_idx = pcl::getFieldIndex (cloud, "z", fields);
//   if (field_z_idx == -1)
//       return (false);
//
//   const size_t offset = fields[field_idx].offset;
//   const size_t z_offset = fields[field_z_idx].offset;
//
//   img.encoding = "rgb8";
//   img.width = cloud.width;
//   img.height = cloud.height;
//   img.step = img.width * sizeof (unsigned char) * 3;
//   img.data.resize (img.step * img.height);
//
//   for (size_t i = 0; i < cloud.points.size (); ++i)
//   {
//     uint32_t val;
//     pcl::getFieldValue<PointT, uint32_t> (cloud.points[i], offset, val);
//     img.data[i * 3 + 0] = (val >> 16) & 0x0000ff;
//     img.data[i * 3 + 1] = (val >> 8) & 0x0000ff;
//     img.data[i * 3 + 2] = (val) & 0x0000ff;
//   }
//
//   return (true);
// }


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
PointCloudImageExtractorGTLabelFromRGBField<PointT>::extractImpl (const PointCloud& cloud, pcl::PCLImage& img) const
{
  char black[3] = {0,0,0};
  std::vector<pcl::PCLPointField> fields;
  int field_idx = pcl::getFieldIndex (cloud, "rgb", fields);
  if (field_idx == -1)
  {
    field_idx = pcl::getFieldIndex (cloud, "rgba", fields);
    if (field_idx == -1)
      return (false);
  }

  int field_z_idx = pcl::getFieldIndex (cloud, "z", fields);
  if (field_z_idx == -1)
      return (false);
      
  const size_t offset = fields[field_idx].offset;
  const size_t z_offset = fields[field_z_idx].offset;

  switch (color_mode_)
  {
    case COLORS_MONO:
    {
      img.encoding = "mono16";
      img.width = cloud.width;
      img.height = cloud.height;
      img.step = img.width * sizeof (unsigned short);
      img.data.resize (img.step * img.height);
      unsigned short* data = reinterpret_cast<unsigned short*>(&img.data[0]);
      for (size_t i = 0; i < cloud.points.size (); ++i)
      {
        
        if(!pcl::isFinite (cloud.points[i])){
            data[i] = static_cast<unsigned short>(background_label_);
        } else {
            uint32_t val;
            pcl::getFieldValue<PointT, uint32_t> (cloud.points[i], z_offset, val);
            // data[i] = static_cast<unsigned short>(val);
            data[i] = static_cast<unsigned short>(foreground_label_);
            
        }
      }
      break;
    }
    case COLORS_RGB_RANDOM:
    {
      img.encoding = "rgb8";
      img.width = cloud.width;
      img.height = cloud.height;
      img.step = img.width * sizeof (unsigned char) * 3;
      img.data.resize (img.step * img.height);
    

      std::srand(std::time(0));
      std::map<uint32_t, size_t> colormap;

      for (size_t i = 0; i < cloud.points.size (); ++i)
      {
        float_t val;
        pcl::getFieldValue<PointT, float_t> (cloud.points[i], z_offset, val);
        if (colormap.count (val) == 0)
        {
          colormap[val] = i * 3;
          img.data[i * 3 + 0] = static_cast<uint8_t> ((std::rand () % 256));
          img.data[i * 3 + 1] = static_cast<uint8_t> ((std::rand () % 256));
          img.data[i * 3 + 2] = static_cast<uint8_t> ((std::rand () % 256));
        }
        else
        {
          memcpy (&img.data[i * 3], &img.data[colormap[val]], 3);
        }
      }
      break;
    }
    case COLORS_RGB_GLASBEY:
    {
      img.encoding = "rgb8";
      img.width = cloud.width;
      img.height = cloud.height;
      img.step = img.width * sizeof (unsigned char) * 3;
      img.data.resize (img.step * img.height);

      std::srand(std::time(0));
      std::set<uint32_t> labels;
      std::map<uint32_t, size_t> colormap;

      // First pass: find unique labels
      /// @todo should a black label be given in the !pcl::isFinite case?
      for (size_t i = 0; i < cloud.points.size (); ++i)
      {
        // If we need to paint NaN points with black do not waste colors on them
        if (paint_nans_with_black_ && !pcl::isFinite (cloud.points[i]))
          continue;
        // uint32_t val;
        // pcl::getFieldValue<PointT, uint32_t> (cloud.points[i], offset, val);
        // labels.insert (val);
        labels.insert(foreground_label_);
      }

      // Assign Glasbey colors in ascending order of labels
      // Note: the color LUT has a finite size (256 colors), therefore when
      // there are more labels the colors will repeat
      //size_t color = 0;
      for (std::set<uint32_t>::iterator iter = labels.begin (); iter != labels.end (); ++iter)
      {
        //colormap[*iter] = color % GlasbeyLUT::size ();
        colormap[*iter] = foreground_label_ % GlasbeyLUT::size ();
        //++color;
      }

      // Second pass: copy colors from the LUT
      for (size_t i = 0; i < cloud.points.size (); ++i)
      {
        
        if (paint_nans_with_black_ && !pcl::isFinite (cloud.points[i]))
        {
            // copy the color 3 channel black memory addresses into the image
            memcpy (&img.data[i * 3], black, 3);        }
        else
        {
            std::size_t offset_into_colormap = colormap[foreground_label_];
            // copy the color 3 channel memory addresses into the image
            memcpy (&img.data[i * 3], GlasbeyLUT::data () + offset_into_colormap * 3, 3);
        }
      }

      break;
    }
  }

  return (true);
}


void
printHelp (int, char **argv)
{
  std::cout << std::endl;
  std::cout << "****************************************************************************" << std::endl;
  std::cout << "*                                                                          *" << std::endl;
  std::cout << "*     PCD 2 PNG SEGMENTATION GROUND TRUTH CONVERTER - Usage Guide          *" << std::endl;
  std::cout << "*                                                                          *" << std::endl;
  std::cout << "****************************************************************************" << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: " << argv[0] << " [Options] input.pcd input_ground_truth.pcd [output_source_data.png output_ground_truth_data.png]" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << std::endl;
  std::cout << "     --help   : Show this help"                                               << std::endl;
  std::cout << "     --nan-class : Paint NaN (infinite) points with their own class color "   << std::endl;
  std::cout << "                this overrides the default of always making nan black"        << std::endl;
  std::cout << "     --foreground-class : The Class ID integer of foreground/nonzero data"    << std::endl;
  std::cout << "                in the ground truth cloud. Supported options:"                << std::endl;
  std::cout << "                - <int>   : id to assign valid ground truth data"             << std::endl;
  std::cout << "     --background-class : The Class ID integer of invalid data in the"        << std::endl;
  std::cout << "                ground truth cloud. Supported options:"                       << std::endl;
  std::cout << "                - <int>   : id to assign valid ground truth data (default: 0)"<< std::endl;
  std::cout << "     --field  : Set the field to extract foreground data from."               << std::endl;
  std::cout << "                Ground truth data is currently always extracted from rgb."    << std::endl;
  std::cout << "                Supported fields:"                                            << std::endl;
  std::cout << "                - normal"                                                     << std::endl;
  std::cout << "                * rgb (default)"                                              << std::endl;
  std::cout << "                - label"                                                      << std::endl;
  std::cout << "                - z"                                                          << std::endl;
  std::cout << "                - curvature"                                                  << std::endl;
  std::cout << "                - intensity"                                                  << std::endl;
  std::cout << "     --colors : Choose color mapping mode for labels (only for label field)." << std::endl;
  std::cout << "                Supported options:"                                           << std::endl;
  std::cout << "                - mono    : Shades of gray representing direct label values." << std::endl;
  std::cout << "                            mono is easiest to load into caffe but has poor"  << std::endl;
  std::cout << "                            visibility because of similar gray color values"  << std::endl;
  std::cout << "                - rgb     : Randomly generated RGB colors"                    << std::endl;
  std::cout << "                * glasbey : Fixed colors from the Glasbey table¹ (default)"   << std::endl;
  std::cout << "                            high visibility colors but labels need to be"     << std::endl;
  std::cout << "                            mapped back to sequential integers in caffe."     << std::endl;
  std::cout << "     --scale  : Apply scaling to extracted data (only for z, curvature, and"  << std::endl;
  std::cout << "                intensity fields). Supported options:"                        << std::endl;
  std::cout << "                - <float> : Scale by a fixed number"                          << std::endl;
  std::cout << "                - auto    : Auto-scale to the full range"                     << std::endl;
  std::cout << "                - no      : No scaling"                                       << std::endl;
  std::cout << "                If the option is omitted then default scaling (depends on"    << std::endl;
  std::cout << "                the field type) will be used."                                << std::endl;
  std::cout << std::endl;
  std::cout << "Notes:"                                                                       << std::endl;
  std::cout << std::endl;
  std::cout << "¹) The Glasbey lookup table is a color table structured in a maximally"       << std::endl;
  std::cout << "   discontinuous manner. Adjacent color bins are chosen to be as distinct"    << std::endl;
  std::cout << "   from one another as possible (see https://github.com/taketwo/glasbey)."    << std::endl;
  std::cout << "   The label with the smallest id will be assigned the first color from the"  << std::endl;
  std::cout << "   table, the second smallest will have the second color, and so on. Thus,"   << std::endl;
  std::cout << "   if you have several clouds with the same labels, you will get repetitive"  << std::endl;
  std::cout << "   consistently colored PNG images."                                          << std::endl;
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud, Eigen::Vector4f& blob_origin, Eigen::Quaternionf& blob_orientation)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud,blob_origin,blob_orientation) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
saveImage (const std::string &filename, const pcl::PCLImage& image)
{
  TicToc tt;
  tt.tic ();
  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  savePNGFile (filename, image);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", image.width * image.height); print_info (" points]\n");
}

template<typename T> bool
parseScaleOption (int argc, char** argv, T& pcie)
{
  std::string scaling = "default";
  pcl::console::parse_argument (argc, argv, "--scale", scaling);
  print_info ("Scaling: "); print_value ("%s\n", scaling.c_str());
  if (scaling == "default")
  {
    // scaling option omitted, use whatever defaults image extractor has
  }
  else if (scaling == "no")
  {
    pcie.setScalingMethod(pcie.SCALING_NO);
  }
  else if (scaling == "auto")
  {
    pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
  }
  else
  {
    try
    {
      float factor = boost::lexical_cast<float> (scaling);
      pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
      pcie.setScalingFactor(factor);
    }
    catch (const boost::bad_lexical_cast&)
    {
      print_error ("The value of --scale option should be \"no\", \"auto\", or a floating point number.\n");
      return false;
    }
  }
  return true;
}

template<typename T> bool
parseColorsOption (int argc, char** argv, T& pcie)
{
  std::string colors = "glasbey";
  pcl::console::parse_argument (argc, argv, "--colors", colors);
  print_info ("Colors: "); print_value ("%s\n", colors.c_str());
  if (colors == "mono")
  {
    pcie.setColorMode(pcie.COLORS_MONO);
  }
  else if (colors == "rgb")
  {
    pcie.setColorMode(pcie.COLORS_RGB_RANDOM);
  }
  else if (colors == "glasbey")
  {
    pcie.setColorMode(pcie.COLORS_RGB_GLASBEY);
  }
  else
  {
    return false;
  }
  return true;
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a PCD file to PNG format.\nFor more information, use: %s --help\n", argv[0]);

  if (argc < 5 || pcl::console::find_switch (argc, argv, "--help"))
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd and .png files
  std::vector<int> pcd_file_index = parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> png_file_index = parse_file_extension_argument (argc, argv, ".png");

  if (pcd_file_index.size () != 2 || (png_file_index.size () != 2 && png_file_index.size() != 0))
  {
    print_error ("Need two input PCD files and two output PNG files.\n");
    return (-1);
  }
  
  std::string png_filename;
  std::string png_gt_filename;
  std::string pcd_filename = argv[pcd_file_index[0]];
  std::string pcd_gt_filename = argv[pcd_file_index[1]];
  if(png_file_index.size() == 0)
  {
    png_filename = pcd_filename;
    png_gt_filename = pcd_gt_filename;
    boost::filesystem::change_extension(png_filename, "png").string();
    boost::filesystem::change_extension(png_gt_filename, "png").string();
  
  } else {
    png_filename = argv[png_file_index[0]];
    png_gt_filename = argv[png_file_index[1]];
  }
  
  int background_class = 0;
  int foreground_class = 1;
  
  parse(argc, argv, "--foreground-class", foreground_class);
  parse(argc, argv, "--background-class", background_class);


  // Load the input file
  pcl::PCLPointCloud2::Ptr blob (new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr blob_gt (new pcl::PCLPointCloud2());
  Eigen::Vector4f blob_origin, blob_gt_origin;
  Eigen::Quaternionf blob_orientation, blob_gt_orientation;
  
  if (!loadCloud (pcd_filename, *blob,blob_origin,blob_orientation))
  {
    print_error ("Unable to load PCD input file 1.\n");
    return (-1);
  }

  if (!loadCloud (pcd_gt_filename, *blob_gt,blob_gt_origin,blob_gt_orientation))
  {
    print_error ("Unable to load PCD ground truth input file 1.\n");
    return (-1);
  }

  // Check if the cloud is organized
  if (blob->height == 1)
  {
    print_error ("Input cloud is not organized.\n");
    return (-1);
  }

  // Check if the cloud is organized
  if (blob_gt->height == 1)
  {
    print_error ("Input ground truth cloud is not organized.\n");
    return (-1);
  }

  bool paint_nans_with_black = !pcl::console::find_switch (argc, argv, "--nan-class");
  print_info ("Paint infinite points with black: "); print_value ("%s\n", paint_nans_with_black ? "YES" : "NO");

  std::string field_name = "rgb";
  parse_argument (argc, argv, "--field", field_name);
  print_info ("Field name: "); print_value ("%s\n", field_name.c_str());


  pcl::PCLImage image;
  bool extracted;
  if (field_name == "normal")
  {
    PointCloud<PointNormal> cloud;
    fromPCLPointCloud2 (*blob, cloud);
    PointCloudImageExtractorFromNormalField<PointNormal> pcie;
    pcie.setPaintNaNsWithBlack (paint_nans_with_black);
    extracted = pcie.extract(cloud, image);
  }
  else if (field_name == "rgb")
  {
    PointCloud<PointXYZRGB> cloud;
    fromPCLPointCloud2 (*blob, cloud);
    PointCloudImageExtractorFromRGBField<PointXYZRGB> pcie;
    pcie.setPaintNaNsWithBlack (paint_nans_with_black);
    extracted = pcie.extract(cloud, image);
  }
  else if (field_name == "label")
  {
    PointCloud<PointXYZL> cloud;
    fromPCLPointCloud2 (*blob, cloud);
    PointCloudImageExtractorFromLabelField<PointXYZL> pcie;
    pcie.setPaintNaNsWithBlack (paint_nans_with_black);
    if (!parseColorsOption(argc, argv, pcie))
      return (-1);
    extracted = pcie.extract(cloud, image);
  }
  else if (field_name == "z")
  {
    PointCloud<PointXYZ> cloud;
    fromPCLPointCloud2 (*blob, cloud);
    PointCloudImageExtractorFromZField<PointXYZ> pcie;
    pcie.setPaintNaNsWithBlack (paint_nans_with_black);
    if (!parseScaleOption(argc, argv, pcie))
      return (-1);
    extracted = pcie.extract(cloud, image);
  }
  else if (field_name == "curvature")
  {
    PointCloud<PointNormal> cloud;
    fromPCLPointCloud2 (*blob, cloud);
    PointCloudImageExtractorFromCurvatureField<PointNormal> pcie;
    pcie.setPaintNaNsWithBlack (paint_nans_with_black);
    if (!parseScaleOption(argc, argv, pcie))
      return (-1);
    extracted = pcie.extract(cloud, image);
  }
  else if (field_name == "intensity")
  {
    PointCloud<PointXYZI> cloud;
    fromPCLPointCloud2 (*blob, cloud);
    PointCloudImageExtractorFromIntensityField<PointXYZI> pcie;
    pcie.setPaintNaNsWithBlack (paint_nans_with_black);
    if (!parseScaleOption(argc, argv, pcie))
      return (-1);
    extracted = pcie.extract(cloud, image);
  }
  else
  {
    print_error ("Unsupported field \"%s\".\n", field_name.c_str());
    return (-1);
  }
  
  int ret = 0;
  
  if (!extracted)
  {
    print_error ("Failed to extract original image from field \"%s\".\n", field_name.c_str());
    ret |=1;
  }
  else
  {
    saveImage (png_filename, image);
  }
  
  
  // Ground Truth image is always extracted from RGB
  pcl::PCLImage gt_image;
  
  {

      PointCloud<PointXYZRGB> gt_cloud;
      fromPCLPointCloud2 (*blob_gt, gt_cloud);
      PointCloudImageExtractorGTLabelFromRGBField<PointXYZRGB> pcie(foreground_class,background_class);
      pcie.setPaintNaNsWithBlack (paint_nans_with_black);
      if (!parseColorsOption(argc, argv, pcie))
        return (-1);
      extracted = pcie.extract(gt_cloud, gt_image);
  }
  
  if (!extracted)
  {
    print_error ("Failed to extract ground truth image from field \"%s\".\n", field_name.c_str());
    ret |=3;
  }

  saveImage (png_gt_filename, gt_image);

  return (ret);
}
