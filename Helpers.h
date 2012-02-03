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

#ifndef HELPERS_H
#define HELPERS_H

// VTK
class vtkPolyData;
class vtkPoints;

// ITK
#include "itkImageRegion.h"
#include "itkIndex.h"

namespace Helpers
{

typedef std::map<itk::Index<2>, unsigned int, itk::Index<2>::LexicographicCompare> CoordinateMapType;
CoordinateMapType ComputeMap(vtkPolyData* const polyData);

void OutputArrayNames(vtkPolyData* const polyData);

float ComputeAverageSpacing(vtkPoints* const points, const unsigned int numberOfPointsToUse);

itk::ImageRegion<2> GetRegionInRadiusAroundPixel(const itk::Index<2>& pixel, const unsigned int radius);

template <typename T>
float ArrayDifference(T* const array1, T* const array2, const unsigned int length);

template<typename TImage>
void DeepCopy(const TImage* const input, TImage* const output);
}

#include "Helpers.hxx"

#endif
