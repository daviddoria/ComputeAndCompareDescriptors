/*=========================================================================
 *
 *  Copyright David Doria 2011 daviddoria@gmail.com
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

#include "Helpers.h"

// VTK
#include <vtkIdList.h>
#include <vtkKdTree.h>
#include <vtkMath.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkProp.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

namespace Helpers
{

void OutputArrayNames(vtkPolyData* const polyData)
{
  vtkIdType numberOfPointArrays = polyData->GetPointData()->GetNumberOfArrays();
  std::cout << "There are " << numberOfPointArrays << " arrays." << std::endl;
  for(vtkIdType i = 0; i < numberOfPointArrays; i++)
    {
    std::cout << polyData->GetPointData()->GetArrayName(i) << std::endl;
    }
}

CoordinateMapType ComputeMap(vtkPolyData* const polyData)
{
  CoordinateMapType coordinateMap;
  vtkIntArray* indexArray = vtkIntArray::SafeDownCast(polyData->GetPointData()->GetArray("OriginalPixel"));
  if(!indexArray)
    {
    throw std::runtime_error("OriginalPixel array must be available to ComputeMap!");
    }

  for(vtkIdType pointId = 0; pointId < polyData->GetNumberOfPoints(); ++pointId)
    {
    //int* pixelIndexArray;
    int pixelIndexArray[2];
    indexArray->GetTupleValue(pointId, pixelIndexArray);

    itk::Index<2> pixelIndex;
    pixelIndex[0] = pixelIndexArray[0];
    pixelIndex[1] = pixelIndexArray[1];
    coordinateMap[pixelIndex] = pointId;
    }
  return coordinateMap;
}


float ComputeAverageSpacing(vtkPoints* points, unsigned int numberOfPointsToUse)
{
  // Compute the average spacing between pairs of closest points in the data.
  // If 'numberOfPointsToUse' is anything positive, only the first 'numberOfPointsToUse'
  // in the data set will be used in the computation. This is an option because often
  // in very large datasets (~1M points) a small sample is enough to determine the average
  // spacing relatively accurately.
  
  if(numberOfPointsToUse == 0)
    {
    numberOfPointsToUse = points->GetNumberOfPoints();
    }
  if(numberOfPointsToUse > static_cast<unsigned int>(points->GetNumberOfPoints()))
    {
    numberOfPointsToUse = points->GetNumberOfPoints();
    }

  float sumOfDistances = 0.;
  //Create the tree
  vtkSmartPointer<vtkKdTree> pointTree = vtkSmartPointer<vtkKdTree>::New();
  pointTree->BuildLocatorFromPoints(points);

  for(vtkIdType i = 0; i < static_cast<vtkIdType>(numberOfPointsToUse); ++i)
    {
    // Get the coordinates of the current point
    double queryPoint[3];
    points->GetPoint(i,queryPoint);
  
    // Find the 2 closest points (the first closest will be exactly the query point)
    vtkSmartPointer<vtkIdList> result = vtkSmartPointer<vtkIdList>::New();
    pointTree->FindClosestNPoints(2, queryPoint, result);
  
    double closestPoint[3];
    points->GetPoint(result->GetId(1), closestPoint);
      
    float squaredDistance = vtkMath::Distance2BetweenPoints(queryPoint, closestPoint);

    // Take the square root to get the Euclidean distance between the points.
    float distance = sqrt(squaredDistance);
    
    sumOfDistances += distance;
    }
    
  float averageDistance = sumOfDistances / static_cast<float>(points->GetNumberOfPoints());
  
  return averageDistance;
}


itk::ImageRegion<2> GetRegionInRadiusAroundPixel(const itk::Index<2>& pixel, const unsigned int radius)
{
  // This function returns a Region with the specified 'radius' centered at 'pixel'. By the definition of the radius of a square patch, the output region is (radius*2 + 1)x(radius*2 + 1).
  // Note: This region is not necessarily entirely inside the image!

  // The "index" is the lower left corner, so we need to subtract the radius from the center to obtain it
  itk::Index<2> lowerLeft;
  lowerLeft[0] = pixel[0] - radius;
  lowerLeft[1] = pixel[1] - radius;

  itk::ImageRegion<2> region;
  region.SetIndex(lowerLeft);
  itk::Size<2> size;
  size[0] = radius*2 + 1;
  size[1] = radius*2 + 1;
  region.SetSize(size);

  return region;
}

} // end namespace
