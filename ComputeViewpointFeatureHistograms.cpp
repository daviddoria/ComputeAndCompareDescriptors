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
#include <vtkStructuredGrid.h>

// PCL
#include <pcl/features/vfh.h>

// Custom
#include "ComputeNormals.h"
#include "Helpers.h"

const std::string ComputeViewpointFeatureHistograms::DescriptorName = "VFH";
