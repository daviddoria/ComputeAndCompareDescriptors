// STL
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> InputCloud;

int main (int argc, char** argv)
{
  if(argc < 5)
    {
    throw std::runtime_error("Required arguments: PCLInputFileName.pcd VTKInputFileName.vtp mask.mha OutputFileName.vtp");
    }

  std::string pclInputFileName = argv[1];
  std::string vtkInputFileName = argv[2];
  std::string maskFileName = argv[3];
  std::string outputFileName = argv[4];
  std::cout << "Reading " << pclInputFileName << " and " << vtkInputFileName << " and " << maskFileName << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pclInputFileName, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  // Read the VTK data
  std::cout << "Reading vtp file..." << std::endl;
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(vtkInputFileName.c_str());
  reader->Update();

  typedef itk::ImageFileReader<MaskImageType> MaskReaderType;
  MaskReaderType::Pointer maskReader = MaskReaderType::New();
  maskReader->SetFileName(maskFileName);
  maskReader->Update();

  OutputCloud::Ptr outputCloud (new OutputCloud);
  ComputeViewpointFeatureHistograms(cloud, maskReader->GetOutput(), reader->GetOutput(), outputCloud);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->DeepCopy(reader->GetOutput());
  AddToPolyData(outputCloud, polyData);

  std::cout << "Writing output..." << std::endl;
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(outputFileName.c_str());
  writer->SetInputConnection(polyData->GetProducerPort());
  writer->Write();

  return 0;
}
