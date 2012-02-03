#ifndef ComputeNormals_h
#define ComputeNormals_h

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

class vtkPolyData;

struct ComputeNormals
{
  typedef pcl::PointCloud<pcl::PointXYZ> InputCloudType;
  typedef pcl::PointCloud<pcl::Normal> OutputCloudType;

  typedef pcl::search::KdTree<pcl::PointXYZ> TreeType;
  TreeType::Ptr Tree;

  void operator()(InputCloudType::Ptr input, OutputCloudType::Ptr output);
  static void AddNormalsToPolyData(OutputCloudType::Ptr cloudWithNormals, vtkPolyData* const polyData);
};

#endif
