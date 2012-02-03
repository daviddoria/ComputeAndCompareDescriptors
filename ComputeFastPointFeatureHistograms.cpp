#include "ComputeFastPointFeatureHistograms.h"
#include "ComputeNormals.h"
// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <pcl/features/fpfh.h>

const std::string ComputeFastPointFeatureHistograms::DescriptorName = "FPFH";

void ComputeFastPointFeatureHistograms::operator()(InputCloud::Ptr input, vtkPolyData* const polyData)
{
  // Setup the feature computation
  pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfhEstimation;
  // Provide the original point cloud (without normals)
  fpfhEstimation.setInputCloud (input);
  // Provide the point cloud with normals
  fpfhEstimation.setInputNormals(input);

  typedef pcl::search::KdTree<InputCloud::PointType> TreeType;
  TreeType::Ptr tree = typename TreeType::Ptr(new TreeType);
  
  // fpfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
  // Use the same KdTree from the normal estimation
  fpfhEstimation.setSearchMethod(tree);

  OutputCloud::Ptr pfhFeatures(new OutputCloud);

  fpfhEstimation.setRadiusSearch (0.2);

  // Actually compute the features
  fpfhEstimation.compute (*pfhFeatures);

  AddToPolyData(pfhFeatures, polyData);
}

void ComputeFastPointFeatureHistograms::AddToPolyData(OutputCloud::Ptr outputCloud, vtkPolyData* const polyData)
{
  std::cout << "Attaching FPFH to VTK data..." << std::endl;

  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName(this->DescriptorName.c_str());
  descriptors->SetNumberOfComponents(33);
  descriptors->SetNumberOfTuples(outputCloud->points.size());

  for(size_t pointId = 0; pointId < outputCloud->points.size(); ++pointId)
    {
    OutputCloud::PointType descriptor = outputCloud->points[pointId];
    descriptors->SetTupleValue(pointId, descriptor.histogram);
    }

  polyData->GetPointData()->AddArray(descriptors);
}
