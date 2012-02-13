#ifndef ComputeViewpointFeatureHistograms_h
#define ComputeViewpointFeatureHistograms_h

// VTK
class vtkPolyData;
class vtkStructuredGrid;

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ITK
#include "itkImage.h"

// Custom
#include "ComputeNormals.h"

class ComputeViewpointFeatureHistograms
{
public:
  static const std::string DescriptorName;
  
  typedef pcl::PointCloud<pcl::PointNormal> InputCloudType;
  typedef pcl::PointCloud<pcl::VFHSignature308> OutputCloudType;

  typedef itk::Image<bool, 2> MaskImageType;

  // Compute descriptors (normals are already computed and input)
  template <typename TVTKObject>
  void operator()(InputCloudType::Ptr input, MaskImageType* mask, TVTKObject* const vtkobject);

  template <typename TVTKObject>
  void AddToPointData(OutputCloudType::Ptr outputCloud, TVTKObject* const polyData);

};

#include "ComputeViewpointFeatureHistograms.hpp"

#endif
