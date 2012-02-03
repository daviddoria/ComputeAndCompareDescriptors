#ifndef ComputeViewpointFeatureHistograms_h
#define ComputeViewpointFeatureHistograms_h

// VTK
class vtkPolyData;

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
  typedef pcl::PointCloud<pcl::PointXYZ> InputCloudType;
  typedef pcl::PointCloud<pcl::VFHSignature308> OutputCloudType;

  typedef itk::Image<bool, 2> MaskImageType;

  // Compute normals and descriptors
  void operator()(InputCloudType::Ptr input, MaskImageType* mask, vtkPolyData* const polyData);

  // Compute descriptors (normals are already computed and input)
  void operator()(ComputeNormals::OutputCloudType::Ptr input, MaskImageType* mask, vtkPolyData* const polyData);

//   void ComputeViewpointFeatureHistogram(InputCloudType::Ptr input, MaskImageType* mask, itk::Index<2>& index,
//                                         OutputCloudType::Ptr output);

  void AddToPolyData(OutputCloudType::Ptr outputCloud, vtkPolyData* const polyData);

};

#endif
