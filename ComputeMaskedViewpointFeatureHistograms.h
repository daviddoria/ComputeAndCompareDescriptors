#ifndef ComputeMaskedViewpointFeatureHistograms_h
#define ComputeMaskedViewpointFeatureHistograms_h

class vtkPolyData;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "itkImage.h"

class ComputeMaskedViewpointFeatureHistograms
{
public:
  typedef pcl::PointCloud<pcl::PointNormal> InputCloudType;
  typedef pcl::PointCloud<pcl::VFHSignature308> OutputCloudType;

  typedef itk::Image<bool, 2> MaskImageType;

  void operator()(InputCloudType::Ptr input, MaskImageType* mask, vtkPolyData* const polyData);

//   void ComputeMaskedViewpointFeatureHistogram(InputCloudType::Ptr input, MaskImageType* mask, itk::Index<2>& index,
//                                               OutputCloudType::Ptr output);

  void AddToPolyData(OutputCloudType::Ptr outputCloud, vtkPolyData* const polyData);

};

#endif
