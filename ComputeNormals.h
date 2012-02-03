#ifndef ComputeNormals_h
#define ComputeNormals_h

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

class vtkPolyData;

template <typename TInput, typename TOutput>
struct ComputeNormals
{
  typedef TInput InputPointType;
  typedef TOutput OutputPointType;

  typedef pcl::search::KdTree<InputPointType> TreeType;
  typename TreeType::Ptr Tree;

  void operator()(typename pcl::PointCloud<TInput>::Ptr input, typename pcl::PointCloud<TOutput>::Ptr output);

  // static void AddNormalsToPolyData(OutputCloudType::Ptr cloudWithNormals, vtkPolyData* const polyData);
};

#include "ComputeNormals.hpp"

#endif
