#include "ComputePFHRGB.h"

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
#include <pcl/features/pfhrgb.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

// Custom
#include "Helpers.h"
#include "ComputeNormals.h"

const std::string ComputePFHRGB::DescriptorName = "PFHRGB";

void ComputePFHRGB::operator()(InputCloudType::Ptr input, MaskImageType* mask, vtkPolyData* const polyData)
{
  // Initalize 'output'
  OutputCloudType::Ptr pfhrgbFeatures(new OutputCloudType);
  pfhrgbFeatures->resize(polyData->GetNumberOfPoints());

  std::cout << "Creating index map..." << std::endl;
  Helpers::CoordinateMapType coordinateMap = Helpers::ComputeMap(polyData);

  unsigned int patch_half_width = 5;

  std::cout << "Computing descriptors..." << std::endl;
  itk::ImageRegion<2> fullRegion = mask->GetLargestPossibleRegion();
  itk::ImageRegionConstIteratorWithIndex<MaskImageType> imageIterator(mask, fullRegion);
  std::cout << "Full region: " << fullRegion << std::endl;

  OutputCloudType::PointType emptyPoint;
  for(unsigned int component = 0; component < 250; ++component)
    {
    emptyPoint.histogram[component] = 0.0f;
    }

  // Create a tree
  typedef pcl::search::KdTree<InputCloudType::PointType> TreeType;
  TreeType::Ptr tree = typename TreeType::Ptr(new TreeType);

  unsigned int counter = 0;
  
  while(!imageIterator.IsAtEnd())
    {
    counter++;
    //std::cout << "Processing point " << counter << " out of " << mask->GetLargestPossibleRegion().GetNumberOfPixels() << std::endl;
    if(counter % 100 == 0)
      {
      std::cout << "Processing point " << counter << " out of " << mask->GetLargestPossibleRegion().GetNumberOfPixels() << std::endl;
      }

    //std::cout << "Computing descriptor for pixel " << imageIterator.GetIndex() << std::endl;
    itk::ImageRegion<2> patchRegion = Helpers::GetRegionInRadiusAroundPixel(imageIterator.GetIndex(), patch_half_width);
    //std::cout << "patchRegion: " << patchRegion << std::endl;
    if(!fullRegion.IsInside(patchRegion))
      {
      ++imageIterator;
      //std::cout << "Skipping patch..." << std::endl;
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

      pfhrgbFeatures->points[currentPointId] = emptyPoint;
      ++imageIterator;
      continue;
      }
    //std::cout << "There are " << pointIds.size() << " points in this patch." << std::endl;

    pcl::ExtractIndices<InputCloudType::PointType> extractIndices;
    extractIndices.setIndices(boost::make_shared<std::vector<int> > (pointIds));
    extractIndices.setInputCloud(input);
    InputCloudType::Ptr extracted(new InputCloudType);
    extractIndices.filter(*extracted);

    //std::cout << "There are " << extracted->points.size() << " extracted points." << std::endl;

    // Setup the feature computation
    pcl::PFHRGBEstimation<InputCloudType::PointType, InputCloudType::PointType, OutputCloudType::PointType> pfhrgbEstimation;

    unsigned int currentPointIdFullCloud = coordinateMap[imageIterator.GetIndex()];
    int currentPointIdExtractedCloud = -1;
    for(unsigned int i = 0; i < pointIds.size(); ++i)
      {
      if(pointIds[i] == currentPointIdFullCloud)
        {
        currentPointIdExtractedCloud = i;
        }
      }
    assert(currentPointIdExtractedCloud != -1); // We should definitely find this point, as it was extracted!

    std::vector<int> centerPointId(1); // We store this as in a vector because normally more than one point is computed, but here we only want the center.
    centerPointId[0] = currentPointIdExtractedCloud;

    pfhrgbEstimation.setIndices(boost::make_shared<std::vector<int> >(centerPointId));

    pfhrgbEstimation.setInputCloud (extracted);
    pfhrgbEstimation.setInputNormals(extracted);

    pfhrgbEstimation.setSearchMethod(tree);

    pfhrgbEstimation.setKSearch(pointIds.size() - 1);

    // Actually compute the VFH for this subset of points
    OutputCloudType::Ptr pfhrgbFeature(new OutputCloudType);
    pfhrgbEstimation.compute (*pfhrgbFeature);

    pfhrgbFeatures->points[currentPointIdFullCloud] = pfhrgbFeature->points[0];

    ++imageIterator;
    } // end while imageIterator

  AddToPolyData(pfhrgbFeatures, polyData);
}

void ComputePFHRGB::AddToPolyData(OutputCloudType::Ptr outputCloud, vtkPolyData* const polyData)
{
  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName(DescriptorName.c_str());
  descriptors->SetNumberOfComponents(250);
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
