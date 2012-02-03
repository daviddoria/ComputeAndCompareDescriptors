#include "ComputeViewpointFeatureHistograms.h"

// Boost
#include <boost/make_shared.hpp>

// ITK
#include "itkImage.h"
#include "itkImageRegionConstIteratorWithIndex.h"

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

// PCL
#include <pcl/features/vfh.h>

// Custom
#include "ComputeNormals.h"
#include "Helpers.h"

const std::string ComputeViewpointFeatureHistograms::DescriptorName = "VFH";

// void ComputeViewpointFeatureHistogram::operator()(InputCloudType::Ptr input, MaskImageType* mask, itk::Index<2>& index,
//                                                   OutputCloudType::Ptr output)
// {
//   // The 'output' of this function only has 1 point (as per the standard operation of the VFH filter).
//   // The inner loop of ComputeViewpointFeatureHistograms() should be broken out here
// }

void ComputeViewpointFeatureHistograms::operator()(InputCloudType::Ptr input, MaskImageType* mask, vtkPolyData* const polyData)
{
  // Only compute the descriptor on a subset of the points
  // Input requirements: 'polyData' must have a vtkIntArray called "OriginalPixel" that has 2-tuples
  // indicating which pixel in the depth image the point corresponds to

  // Initalize 'output'
  OutputCloudType::Ptr output(new OutputCloudType);
  output->resize(polyData->GetNumberOfPoints());

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

  // Create a tree
  typedef pcl::search::KdTree<InputCloudType::PointType> TreeType;
  TreeType::Ptr tree = typename TreeType::Ptr(new TreeType);

  //std::fstream fout("/home/doriad/temp/output.txt");
  unsigned int counter = 0;

  while(!imageIterator.IsAtEnd())
    {
    counter++;
    if(counter % 1000 == 0)
      {
      std::cout << "Processing point " << counter << " out of " << mask->GetLargestPossibleRegion().GetNumberOfPixels() << std::endl;
      }
    //std::cout << "Computing descriptor for pixel " << imageIterator.GetIndex() << std::endl;
    //fout << "Computing descriptor for pixel " << imageIterator.GetIndex() << std::endl;
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

      output->points[currentPointId] = emptyPoint;
      ++imageIterator;
      continue;
      }
    //std::cout << "There are " << pointIds.size() << " points in this patch." << std::endl;

    // Setup the feature computation
    pcl::VFHEstimation<InputCloudType::PointType, pcl::PointNormal, OutputCloudType::PointType> vfhEstimation;

    //vfhEstimation.setIndices(&pointIds);
    vfhEstimation.setIndices(boost::make_shared<std::vector<int> >(pointIds));

    // Provide the original point cloud (without normals)
    vfhEstimation.setInputCloud (input);

    // Provide the point cloud with normals
    vfhEstimation.setInputNormals(input);

    // vfhEstimation.setInputWithNormals(cloud, cloudWithNormals); VFHEstimation does not have this function
    // Use the same KdTree from the normal estimation
    vfhEstimation.setSearchMethod (tree);

    //vfhEstimation.setRadiusSearch (0.2);

    // Actually compute the VFH for this subset of points
    OutputCloudType::Ptr vfhFeature(new OutputCloudType);
    vfhEstimation.compute (*vfhFeature);

    unsigned int currentPointId = coordinateMap[imageIterator.GetIndex()];

    output->points[currentPointId] = vfhFeature->points[0];

    ++imageIterator;
    } // end while imageIterator

  AddToPolyData(output, polyData);
}

void ComputeViewpointFeatureHistograms::AddToPolyData(OutputCloudType::Ptr outputCloud, vtkPolyData* const polyData)
{
  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName(this->DescriptorName.c_str());
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
