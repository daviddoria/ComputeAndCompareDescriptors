#include "ComputeNormals.h"

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <pcl/features/normal_3d.h>

ComputeNormals::ComputeNormals(InputCloud::Ptr input, OutputCloud::Ptr output)
{
  Compute(input, output);
}

void ComputeNormals::Compute(InputCloud::Ptr input, OutputCloud::Ptr output)
{
  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (input);

  this->Tree = TreeType::Ptr(new TreeType);
  normalEstimation.setSearchMethod (this->Tree);

  normalEstimation.setRadiusSearch (0.1);

  normalEstimation.compute (*output);
}

void ComputeNormals::AddNormalsToPolyData(OutputCloud::Ptr cloudWithNormals, vtkPolyData* const polyData)
{
  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName("Normals");
  descriptors->SetNumberOfComponents(3);
  descriptors->SetNumberOfTuples(cloudWithNormals->points.size());

  std::cout << "Attaching features to VTK data..." << std::endl;
  for(size_t pointId = 0; pointId < cloudWithNormals->points.size(); ++pointId)
    {
    pcl::Normal descriptor = cloudWithNormals->points[pointId];
    descriptors->SetTupleValue(pointId, descriptor.data_n); // Is this the right member?
    }

  polyData->GetPointData()->AddArray(descriptors);

}
