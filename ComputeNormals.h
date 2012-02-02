#ifndef ComputeNormals_h
#define ComputeNormals_h

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

class vtkPolyData;

struct ComputeNormals
{
  typedef pcl::PointCloud<pcl::PointXYZ> InputCloud;
  typedef pcl::PointCloud<pcl::Normal> OutputCloud;

  typedef pcl::search::KdTree<pcl::PointXYZ> TreeType;
  TreeType::Ptr Tree;

  ComputeNormals(){}
  ComputeNormals(InputCloud::Ptr input, OutputCloud::Ptr output);
  void Compute(InputCloud::Ptr input, OutputCloud::Ptr output);
  
  static void AddNormalsToPolyData(OutputCloud::Ptr cloudWithNormals, vtkPolyData* const polyData);
};

#endif
