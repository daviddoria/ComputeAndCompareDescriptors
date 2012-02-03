#ifndef ComputeFastPointFeatureHistograms_h
#define ComputeFastPointFeatureHistograms_h


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class vtkPolyData;

class ComputeFastPointFeatureHistograms
{
public:
  static const std::string DescriptorName;

  typedef pcl::PointCloud<pcl::PointNormal> InputCloud;
  typedef pcl::PointCloud<pcl::FPFHSignature33> OutputCloud;

  void operator()(InputCloud::Ptr input, vtkPolyData* const polyData);
  void AddToPolyData(OutputCloud::Ptr outputCloud, vtkPolyData* const polyData);

};

#endif
