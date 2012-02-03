#ifndef ComputePointFeatureHistograms_h
#define ComputePointFeatureHistograms_h

// VTK
class vtkPolyData;

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ITK
#include "itkImage.h"

// Custom
#include "ComputeNormals.h"

class ComputePointFeatureHistograms
{
public:
  static const std::string DescriptorName;
  
  // The input must already have normals
  typedef pcl::PointCloud<pcl::PointNormal> InputCloudType;
  typedef pcl::PointCloud<pcl::PFHSignature125> OutputCloudType;

  typedef itk::Image<bool, 2> MaskImageType;

  // Compute descriptors (normals are already computed and input)
  void operator()(InputCloudType::Ptr input, MaskImageType* mask, vtkPolyData* const polyData);

//   void ComputeViewpointFeatureHistogram(InputCloudType::Ptr input, MaskImageType* mask, itk::Index<2>& index,
//                                         OutputCloudType::Ptr output);

  void AddToPolyData(OutputCloudType::Ptr outputCloud, vtkPolyData* const polyData);

};

#endif
