#include "ComputePointFeatureHistograms.h"

// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/impl/pfh.hpp>
#include <pcl/features/normal_3d.h>

// Boost
#include <boost/make_shared.hpp>

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

const std::string ComputePointFeatureHistograms::DescriptorName = "PFH";

// The input must already have normals.
void ComputePointFeatureHistograms::operator()(InputCloudType::Ptr input, vtkPolyData* const polyData)
{
  pcl::search::KdTree<InputCloudType::PointType>::Ptr tree (new pcl::search::KdTree<InputCloudType::PointType>);

  // Setup the feature computation
  pcl::PFHEstimation<InputCloudType::PointType, InputCloudType::PointType, OutputCloudType::PointType> pfhEstimation;
  // Provide the original point cloud (without normals)
  pfhEstimation.setInputCloud (input);
  // Provide the point cloud with normals
  pfhEstimation.setInputNormals(input);

  // Use the same KdTree from the normal estimation
  pfhEstimation.setSearchMethod (tree);

  OutputCloudType::Ptr pfhFeatures(new OutputCloudType);

//   float radius = .1f;
//   pfhEstimation.setRadiusSearch (radius);
  
  // This is here for testing only - it only computes the descriptor on a specified number of points - it is used to do some benchmarking.
//   {
//   std::vector<int> indices;
//   for(unsigned int i = 0; i < 1000; ++i)
//   {
//     indices.push_back(i);
//   }
//   pfhEstimation.setIndices(boost::make_shared<std::vector<int> >(indices));
//   }
  
  //unsigned int numberOfNeighbors = 100;
  // unsigned int numberOfNeighbors = 5; // takes only a few seconds
  //unsigned int numberOfNeighbors = 10; // takes about 2 minutes
  unsigned int numberOfNeighbors = 20;
  pfhEstimation.setKSearch(numberOfNeighbors);

  // Actually compute the features
  std::cout << "Computing features..." << std::endl;
  pfhEstimation.compute (*pfhFeatures);

  std::cout << "output points (features computed on): " << pfhFeatures->points.size () << std::endl;
  
  AddToPolyData(pfhFeatures, polyData);
}

void ComputePointFeatureHistograms::AddToPolyData(OutputCloudType::Ptr outputCloud, vtkPolyData* const polyData)
{
  std::cout << "Attaching PFH features to VTK data..." << std::endl;

  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName(this->DescriptorName.c_str());
  descriptors->SetNumberOfComponents(125);
  descriptors->SetNumberOfTuples(polyData->GetNumberOfPoints());

  // Zero all of the descriptors, we may not have one to assign for every point.
  std::vector<float> zeroVector(125, 0);

  for(size_t pointId = 0; pointId < polyData->GetNumberOfPoints(); ++pointId)
    {
    descriptors->SetTupleValue(pointId, zeroVector.data());
    }

  for(size_t pointId = 0; pointId < outputCloud->points.size(); ++pointId)
    {
    OutputCloudType::PointType descriptor = outputCloud->points[pointId];
    descriptors->SetTupleValue(pointId, descriptor.histogram);
    }

  polyData->GetPointData()->AddArray(descriptors);

}
