#include "ComputeFastPointFeatureHistograms.h"
#include "ComputeNormals.h"
// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <pcl/features/fpfh.h>

namespace ComputeFastPointFeatureHistograms
{

void ComputeFastPointFeatureHistograms(InputCloud::Ptr input, OutputCloud::Ptr output)
{
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//   normalEstimation.setSearchMethod (tree);

  ComputeNormals::OutputCloud::Ptr cloudWithNormals (new ComputeNormals::OutputCloud);
  ComputeNormals normals(input, cloudWithNormals);
  
  // Setup the feature computation
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;
  // Provide the original point cloud (without normals)
  fpfhEstimation.setInputCloud (input);
  // Provide the point cloud with normals
  fpfhEstimation.setInputNormals(cloudWithNormals);

  // fpfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
  // Use the same KdTree from the normal estimation
  fpfhEstimation.setSearchMethod (normals.Tree);

  OutputCloud::Ptr pfhFeatures(new OutputCloud);

  fpfhEstimation.setRadiusSearch (0.2);

  // Actually compute the spin images
  fpfhEstimation.compute (*pfhFeatures);

  std::cout << "output points.size (): " << pfhFeatures->points.size () << std::endl;

  // Display and retrieve the shape context descriptor vector for the 0th point.
  pcl::FPFHSignature33 descriptor = pfhFeatures->points[0];
  std::cout << descriptor << std::endl;
}

void AddToPolyData(OutputCloud::Ptr outputCloud, vtkPolyData* const polyData)
{
  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName("Normals");
  descriptors->SetNumberOfComponents(3);
  descriptors->SetNumberOfTuples(outputCloud->points.size());

  std::cout << "Attaching features to VTK data..." << std::endl;
  for(size_t pointId = 0; pointId < outputCloud->points.size(); ++pointId)
    {
    OutputCloud::PointType descriptor = outputCloud->points[pointId];
    descriptors->SetTupleValue(pointId, descriptor.histogram);
    }

  polyData->GetPointData()->AddArray(descriptors);
}

}
