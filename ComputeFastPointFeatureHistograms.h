#ifndef ComputeFastPointFeatureHistograms_h
#define ComputeFastPointFeatureHistograms_h


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class vtkPolyData;

namespace ComputeFastPointFeatureHistograms
{

typedef pcl::PointCloud<pcl::PointXYZ> InputCloud;
typedef pcl::PointCloud<pcl::FPFHSignature33> OutputCloud;

void ComputeFastPointFeatureHistograms(InputCloud::Ptr input, OutputCloud::Ptr output);
void AddToPolyData(OutputCloud::Ptr outputCloud, vtkPolyData* const polyData);

}

#endif
