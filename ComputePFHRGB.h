#ifndef ComputePFHRGB_h
#define ComputePFHRGB_h

class vtkPolyData;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "itkImage.h"

class ComputePFHRGB
{
public:

  static const std::string DescriptorName;

  typedef pcl::PointCloud<pcl::PointXYZRGBNormal> InputCloudType;

  typedef pcl::PFHRGBSignature250 FeatureType;
  typedef pcl::PointCloud<FeatureType> OutputCloudType;

  typedef itk::Image<bool, 2> MaskImageType;

  void operator()(InputCloudType::Ptr input, MaskImageType* mask, vtkPolyData* const polyData);

  void AddToPolyData(OutputCloudType::Ptr outputCloud, vtkPolyData* const polyData);

};

#endif
