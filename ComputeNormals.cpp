#include "ComputeNormals.h"

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <pcl/features/normal_3d.h>
#include <pcl/common/io.h>

// void ComputeNormals::AddNormalsToPolyData(OutputCloudType::Ptr cloudWithNormals, vtkPolyData* const polyData)
// {
//   vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
//   descriptors->SetName("Normals");
//   descriptors->SetNumberOfComponents(3);
//   descriptors->SetNumberOfTuples(cloudWithNormals->points.size());
// 
//   std::cout << "Attaching features to VTK data..." << std::endl;
//   for(size_t pointId = 0; pointId < cloudWithNormals->points.size(); ++pointId)
//     {
//     OutputCloudType::PointType descriptor = cloudWithNormals->points[pointId];
//     descriptors->SetTupleValue(pointId, descriptor.data_n); // Is this the right member?
//     }
// 
//   polyData->GetPointData()->AddArray(descriptors);
// 
// }
