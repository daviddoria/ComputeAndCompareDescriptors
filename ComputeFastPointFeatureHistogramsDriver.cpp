// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

#include "ComputeFastPointFeatureHistograms.h"

int main (int argc, char** argv)
{
  if(argc < 4)
  {
    throw std::runtime_error("Required arguments: filename.pcd  VTKFileName.vtp OutputFileName.vtp");
  }

  std::string fileName = argv[1];
  std::string vtkFileName = argv[2];
  std::string outputFileName = argv[3];
  std::cout << "Reading " << fileName << std::endl;

  ComputeFastPointFeatureHistograms::InputCloud::Ptr cloud (new ComputeFastPointFeatureHistograms::InputCloud);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(vtkFileName.c_str());
  reader->Update();

  ComputeFastPointFeatureHistograms::OutputCloud::Ptr outputCloud (new ComputeFastPointFeatureHistograms::OutputCloud);
  ComputeFastPointFeatureHistograms::ComputeFastPointFeatureHistograms(cloud, outputCloud);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->DeepCopy(reader->GetOutput());
  ComputeFastPointFeatureHistograms::AddToPolyData(outputCloud, polyData);

  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(outputFileName.c_str());
  writer->SetInputConnection(polyData->GetProducerPort());
  writer->Write();

  return 0;
}
