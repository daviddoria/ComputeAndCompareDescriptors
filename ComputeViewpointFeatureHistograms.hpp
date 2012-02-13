#ifndef ComputeViewpointFeatureHistograms_hpp
#define ComputeViewpointFeatureHistograms_hpp

#include <vtkFloatArray.h>
#include <vtkSmartPointer.h>

template <typename TObject>
void ComputeViewpointFeatureHistograms::AddToPointData(OutputCloudType::Ptr outputCloud, TObject* const vtkobject)
{
  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName(this->DescriptorName.c_str());
  descriptors->SetNumberOfComponents(308);
  descriptors->SetNumberOfTuples(vtkobject->GetNumberOfPoints());

//   // Zero all of the descriptors, we may not have one to assign for every point.
//   std::vector<float> zeroVector(308, 0);
//
//   for(size_t pointId = 0; pointId < outputCloud->points.size(); ++pointId)
//     {
//     descriptors->SetTupleValue(pointId, zeroVector.data());
//     }

  for(vtkIdType pointId = 0; pointId < vtkobject->GetNumberOfPoints(); ++pointId)
    {
    descriptors->SetTupleValue(pointId, outputCloud->points[pointId].histogram);
    }

  vtkobject->GetPointData()->AddArray(descriptors);
}

#endif
