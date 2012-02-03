#include "ComputeClusteredViewpointFeatureHistograms.h"

// STL
#include <iostream>
#include <vector>

// ITK
#include "itkImageRegionConstIteratorWithIndex.h"

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

// Boost
#include <boost/make_shared.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/normal_3d.h>

// Custom
#include "Helpers.h"
#include "ComputeNormals.h"

void ComputeClusteredViewpointFeatureHistograms::operator()(InputCloudType::Ptr input,
                                                            MaskImageType* mask, vtkPolyData* const polyData)
{
  // Initalize 'output'
  OutputCloudType::Ptr cvfhFeatures(new OutputCloudType);
  cvfhFeatures->resize(polyData->GetNumberOfPoints());

  ComputeNormals::OutputCloudType::Ptr cloudWithNormals (new ComputeNormals::OutputCloudType);
  ComputeNormals normals;
  normals(input, cloudWithNormals);

  std::cout << "Creating index map..." << std::endl;
  Helpers::CoordinateMapType coordinateMap = Helpers::ComputeMap(polyData);

  unsigned int patch_half_width = 5;

  std::cout << "Computing descriptors..." << std::endl;
  itk::ImageRegion<2> fullRegion = mask->GetLargestPossibleRegion();
  itk::ImageRegionConstIteratorWithIndex<MaskImageType> imageIterator(mask, fullRegion);
  std::cout << "Full region: " << fullRegion << std::endl;

  OutputCloudType::PointType emptyPoint;
  for(unsigned int component = 0; component < 308; ++component)
    {
    emptyPoint.histogram[component] = 0.0f;
    }

  while(!imageIterator.IsAtEnd())
    {
    //std::cout << "Computing descriptor for pixel " << imageIterator.GetIndex() << std::endl;
    itk::ImageRegion<2> patchRegion = Helpers::GetRegionInRadiusAroundPixel(imageIterator.GetIndex(), patch_half_width);
    //std::cout << "patchRegion: " << patchRegion << std::endl;
    if(!fullRegion.IsInside(patchRegion))
      {
      ++imageIterator;
      continue;
      }

    // Get a list of the pointIds in the region
    //std::vector<unsigned int> pointIds;
    std::vector<int> pointIds;

    itk::ImageRegionConstIteratorWithIndex<MaskImageType> patchIterator(mask, patchRegion);
    while(!patchIterator.IsAtEnd())
      {
      if(!patchIterator.Get())
        {
        pointIds.push_back(coordinateMap[patchIterator.GetIndex()]);
        }
      ++patchIterator;
      }

    if(pointIds.size() < 2)
      {
      unsigned int currentPointId = coordinateMap[imageIterator.GetIndex()];

      cvfhFeatures->points[currentPointId] = emptyPoint;
      ++imageIterator;
      continue;
      }
    //std::cout << "There are " << pointIds.size() << " points in this patch." << std::endl;

    // Setup the feature computation
    pcl::CVFHEstimation<InputCloudType::PointType, pcl::Normal, OutputCloudType::PointType> cvfhEstimation;

    //cvfhEstimation.setIndices(&pointIds);
    cvfhEstimation.setIndices(boost::make_shared<std::vector<int> >(pointIds));

    // Provide the original point cloud (without normals)
    cvfhEstimation.setInputCloud (input);

    // Provide the point cloud with normals
    cvfhEstimation.setInputNormals(cloudWithNormals);

    // cvfhEstimation.setInputWithNormals(cloud, cloudWithNormals); VFHEstimation does not have this function
    // Use the same KdTree from the normal estimation
    cvfhEstimation.setSearchMethod (normals.Tree);

    //cvfhEstimation.setRadiusSearch (0.2);

    // Actually compute the VFH for this subset of points
    OutputCloudType::Ptr cvfhFeature(new OutputCloudType);
    cvfhEstimation.compute (*cvfhFeature);

    unsigned int currentPointId = coordinateMap[imageIterator.GetIndex()];

    cvfhFeatures->points[currentPointId] = cvfhFeature->points[0];

    ++imageIterator;
    } // end while imageIterator

  AddToPolyData(cvfhFeatures, polyData);
}

void ComputeClusteredViewpointFeatureHistograms::AddToPolyData(OutputCloudType::Ptr outputCloud, vtkPolyData* const polyData)
{
  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName("ViewpointFeatureHistograms");
  descriptors->SetNumberOfComponents(308);
  descriptors->SetNumberOfTuples(polyData->GetNumberOfPoints());

//   // Zero all of the descriptors, we may not have one to assign for every point.
//   std::vector<float> zeroVector(308, 0);
//
//   for(size_t pointId = 0; pointId < outputCloud->points.size(); ++pointId)
//     {
//     descriptors->SetTupleValue(pointId, zeroVector.data());
//     }

  for(vtkIdType pointId = 0; pointId < polyData->GetNumberOfPoints(); ++pointId)
    {
    descriptors->SetTupleValue(pointId, outputCloud->points[pointId].histogram);
    }

  polyData->GetPointData()->AddArray(descriptors);
}
