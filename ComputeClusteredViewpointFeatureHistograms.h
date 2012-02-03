#ifndef ComputeClusteredViewpointFeatureHistograms_h
#define ComputeClusteredViewpointFeatureHistograms_h

class vtkPolyData;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "itkImage.h"

class ComputeClusteredViewpointFeatureHistograms
{
public:

  static const std::string DescriptorName;

  typedef pcl::PointCloud<pcl::PointNormal> InputCloudType;

  typedef pcl::VFHSignature308 FeatureType;
  typedef pcl::PointCloud<FeatureType> OutputCloudType;

  typedef itk::Image<bool, 2> MaskImageType;

  void operator()(InputCloudType::Ptr input, MaskImageType* mask, vtkPolyData* const polyData);

  void ComputeViewpointFeatureHistogram(InputCloudType::Ptr input, MaskImageType* mask,
                                        itk::Index<2>& index, OutputCloudType::Ptr output);

  void AddToPolyData(OutputCloudType::Ptr outputCloud, vtkPolyData* const polyData);

};

#endif
