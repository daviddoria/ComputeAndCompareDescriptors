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

#include "ui_ComputeAndCompareDescriptorsWidget.h"
#include "ComputeAndCompareDescriptorsWidget.h"

// ITK
#include "itkCastImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkRegionOfInterestImageFilter.h"
//#include "itkVector.h"

// Qt
#include <QFileDialog>
#include <QIcon>
#include <QProgressDialog>
#include <QTextEdit>
#include <QtConcurrentRun>

// VTK
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkExtractSelectedIds.h>
#include <vtkFloatArray.h>
#include <vtkIdTypeArray.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleImage.h>
#include <vtkLookupTable.h>
#include <vtkMath.h>
#include <vtkPointData.h>
#include <vtkPointPicker.h>
#include <vtkProperty2D.h>
#include <vtkPolyDataMapper.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkSmartPointer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

// PCL
#include <pcl/features/vfh.h>

// Boost
#include <boost/make_shared.hpp>

// Custom
#include "Helpers.h"
#include "Types.h"
#include "PointSelectionStyle3D.h"
#include "VTKtoPCL.h"
#include "ComputeNormals.h"

void ComputeAndCompareDescriptorsWidget::on_actionHelp_activated()
{
  QTextEdit* help=new QTextEdit();

  help->setReadOnly(true);
  help->append("<h1>Compare descriptors</h1>\
  Load a point cloud <br/>\
  Ctrl+click to select a point. <br/>\
  Click Compare.<br/>"
  );
  help->show();
}

void ComputeAndCompareDescriptorsWidget::on_actionQuit_activated()
{
  exit(0);
}

// Constructor
ComputeAndCompareDescriptorsWidget::ComputeAndCompareDescriptorsWidget() :
MarkerRadius(.05), PCLCloud(new InputCloudType), PCLCloudWithNormals(new NormalsCloudType)
{
  this->ProgressDialog = new QProgressDialog();
  SharedConstructor();
};

void ComputeAndCompareDescriptorsWidget::SharedConstructor()
{
  this->setupUi(this);

  this->Mask = MaskImageType::New();

  //this->PCLCloud = boost::make_shared<InputCloud>(*(new InputCloud));
  //this->PCLCloudWithNormals = boost::make_shared<NormalsCloud>(*(new NormalsCloud));
  
  this->PointPicker = vtkSmartPointer<vtkPointPicker>::New();
  this->PointPicker->PickFromListOn();

  // Point cloud
  this->PointCloud = vtkSmartPointer<vtkPolyData>::New();

  this->PointCloudMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  this->PointCloudMapper->SetInputConnection(this->PointCloud->GetProducerPort());

  this->PointCloudActor = vtkSmartPointer<vtkActor>::New();
  this->PointCloudActor->SetMapper(this->PointCloudMapper);

  // Marker
  this->MarkerSource = vtkSmartPointer<vtkSphereSource>::New();
  this->MarkerSource->SetRadius(this->MarkerRadius);
  this->MarkerSource->Update();

  this->MarkerMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  this->MarkerMapper->SetInputConnection(this->MarkerSource->GetOutputPort());

  this->MarkerActor = vtkSmartPointer<vtkActor>::New();
  this->MarkerActor->SetMapper(this->MarkerMapper);

  // Renderer
  this->Renderer = vtkSmartPointer<vtkRenderer>::New();
  this->Renderer->AddActor(this->PointCloudActor);
  this->Renderer->AddActor(this->MarkerActor);

  this->SelectionStyle = PointSelectionStyle3D::New();
  this->SelectionStyle->AddObserver(this->SelectionStyle->SelectedPointEvent, this, &ComputeAndCompareDescriptorsWidget::SelectedPointCallback);

  // Qt things
  this->qvtkWidget->GetRenderWindow()->AddRenderer(this->Renderer);

  connect(&this->FutureWatcher, SIGNAL(finished()), this->ProgressDialog , SLOT(cancel()));

  // Setup icons
  QIcon openIcon = QIcon::fromTheme("document-open");

  actionOpenPointCloud->setIcon(openIcon);
  this->toolBar_left->addAction(actionOpenPointCloud);
}

void ComputeAndCompareDescriptorsWidget::SelectedPointCallback(vtkObject* caller, long unsigned int eventId, void* callData)
{
  double p[3];
  this->PointCloud->GetPoint(this->SelectionStyle->SelectedPointId, p);
  this->MarkerActor->SetPosition(p);
}

void ComputeAndCompareDescriptorsWidget::Refresh()
{
  this->qvtkWidget->GetRenderWindow()->Render();
}

void ComputeAndCompareDescriptorsWidget::LoadMask(const std::string& fileName)
{
  std::cout << "Reading mask " << fileName << std::endl;
  
  typedef itk::ImageFileReader<MaskImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(fileName);
  reader->Update();

  Helpers::DeepCopy(reader->GetOutput(), this->Mask.GetPointer());
}

void ComputeAndCompareDescriptorsWidget::LoadPointCloud(const std::string& fileName)
{
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(fileName.c_str());
  reader->Update();

  // Keep all points
  //this->PointCloud->DeepCopy(reader->GetOutput());
  
  // Keep only unmasked points
  vtkSmartPointer<vtkIdTypeArray> idsToKeep = vtkSmartPointer<vtkIdTypeArray>::New();
  idsToKeep->SetNumberOfComponents(1);

  vtkIntArray* indexArray = vtkIntArray::SafeDownCast(reader->GetOutput()->GetPointData()->GetArray("OriginalPixel"));

  // Keep only unmasked points
  for(vtkIdType pointId = 0; pointId < reader->GetOutput()->GetNumberOfPoints(); ++pointId)
    {
    int pixelIndexArray[2];
    indexArray->GetTupleValue(pointId, pixelIndexArray);

    itk::Index<2> pixelIndex;
    pixelIndex[0] = pixelIndexArray[0];
    pixelIndex[1] = pixelIndexArray[1];

    if(!Mask->GetPixel(pixelIndex))
      {
      idsToKeep->InsertNextValue(pointId);
      }
    }

  vtkSmartPointer<vtkSelectionNode> selectionNode =
    vtkSmartPointer<vtkSelectionNode>::New();
  selectionNode->SetFieldType(vtkSelectionNode::POINT);
  selectionNode->SetContentType(vtkSelectionNode::INDICES);
  selectionNode->SetSelectionList(idsToKeep);
 
  vtkSmartPointer<vtkSelection> selection = vtkSmartPointer<vtkSelection>::New();
  selection->AddNode(selectionNode);
 
  vtkSmartPointer<vtkExtractSelectedIds> extractSelectedIds = vtkSmartPointer<vtkExtractSelectedIds>::New();
  extractSelectedIds->SetInputConnection(0, reader->GetOutputPort());
  extractSelectedIds->SetInput(1, selection);
  extractSelectedIds->Update();
 
  // Convert back to polydata
  vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter = 
    vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
  surfaceFilter->SetInputConnection(extractSelectedIds->GetOutputPort());
  surfaceFilter->Update(); 
  this->PointCloud->DeepCopy(surfaceFilter->GetOutput());

  this->PointCloudMapper->SetInputConnection(this->PointCloud->GetProducerPort());

  this->PointCloudActor->GetProperty()->SetRepresentationToPoints();

  this->Renderer->ResetCamera();

  this->PointPicker->AddPickList(this->PointCloudActor);

  this->qvtkWidget->GetRenderWindow()->GetInteractor()->SetPicker(this->PointPicker);
  this->SelectionStyle->Points = this->PointCloud;
  this->SelectionStyle->SetCurrentRenderer(this->Renderer);
  this->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(this->SelectionStyle);

  this->Renderer->ResetCamera();

  std::cout << "Converting to PCL..." << std::endl;
  //VTKtoPCL(this->PointCloud.GetPointer(), this->PCLCloud.get());
  VTKtoPCL(this->PointCloud, this->PCLCloud.get());

  std::cout << "Computing normals..." << std::endl;
  this->NormalComputer(this->PCLCloud, this->PCLCloudWithNormals);

  CreateIndexMap();

}

void ComputeAndCompareDescriptorsWidget::on_btnCompute_clicked()
{
  // Start the computation.
  QFuture<void> future = QtConcurrent::run(this, &ComputeAndCompareDescriptorsWidget::ComputeDifferences);
  this->FutureWatcher.setFuture(future);
  this->ProgressDialog->setMinimum(0);
  this->ProgressDialog->setMaximum(0);
  this->ProgressDialog->setLabelText("Computing descriptors and differences...");
  this->ProgressDialog->setWindowModality(Qt::WindowModal);
  this->ProgressDialog->exec();
 
  // ComputeDifferences();
}

void ComputeAndCompareDescriptorsWidget::CreateIndexMap()
{
  std::cout << "Creating index map..." << std::endl;

  // Add a map from all of the pixels to their corresponding point id. 
  vtkIntArray* indexArray = vtkIntArray::SafeDownCast(this->PointCloud->GetPointData()->GetArray("OriginalPixel"));
  for(vtkIdType pointId = 0; pointId < this->PointCloud->GetNumberOfPoints(); ++pointId)
    {
    //int* pixelIndexArray;
    int pixelIndexArray[2];
    indexArray->GetTupleValue(pointId, pixelIndexArray);

    itk::Index<2> pixelIndex;
    pixelIndex[0] = pixelIndexArray[0];
    pixelIndex[1] = pixelIndexArray[1];
    this->CoordinateMap[pixelIndex] = pointId;
    }

}

void ComputeAndCompareDescriptorsWidget::ComputeFeatures()
{
  // Only compute the descriptor on a subset of the points
  // Input requirements: 'polyData' must have a vtkIntArray called "OriginalPixel" that has 2-tuples indicating which pixel in the depth image the point corresponds to

  std::cout << "There are " << this->PointCloud->GetNumberOfPoints() << " vtp points." << std::endl;

  // Initalize 'output'
  //OutputCloud::Ptr featureCloud = boost::make_shared<OutputCloud>(*(new OutputCloud)); // This works, but maybe a bad idea?
  OutputCloudType::Ptr featureCloud(new OutputCloudType);
  featureCloud->resize(this->PointCloud->GetNumberOfPoints());

  unsigned int patch_half_width = 10;

  vtkIntArray* indexArray = vtkIntArray::SafeDownCast(this->PointCloud->GetPointData()->GetArray("OriginalPixel"));
  if(!indexArray)
    {
    throw std::runtime_error("OriginalPixel array must be available to ComputeFeatures()!");
    }

  vtkIdType selectedPointId = this->SelectionStyle->SelectedPointId;
  int selectedPixelArray[2];
  indexArray->GetTupleValue(selectedPointId, selectedPixelArray);
  itk::Index<2> selectedPixel = {{selectedPixelArray[0], selectedPixelArray[1]}};
  itk::ImageRegion<2> selectedRegion = Helpers::GetRegionInRadiusAroundPixel(selectedPixel, patch_half_width);

  // Create the list of valid offsets
  std::vector<itk::Offset<2> > validOffsets;
  itk::ImageRegionConstIteratorWithIndex<MaskImageType> patchIterator(this->Mask, selectedRegion);
  while(!patchIterator.IsAtEnd())
    {
    if(!patchIterator.Get())
      {
      validOffsets.push_back(patchIterator.GetIndex() - selectedRegion.GetIndex());
      }
    ++patchIterator;
    }

  std::cout << "Computing descriptors..." << std::endl;
  itk::ImageRegion<2> fullRegion = this->Mask->GetLargestPossibleRegion();
  itk::ImageRegionConstIteratorWithIndex<MaskImageType> imageIterator(this->Mask, fullRegion);
  std::cout << "Full region: " << fullRegion << std::endl;

  OutputCloudType::PointType emptyPoint;
  for(unsigned int component = 0; component < 308; ++component)
    {
    emptyPoint.histogram[component] = 0.0f;
    }

  //std::fstream fout("/home/doriad/temp/output.txt");
  unsigned int numberOfPointsProcessed = 0;
  while(!imageIterator.IsAtEnd())
    {
    numberOfPointsProcessed++;
    if(numberOfPointsProcessed % 1000 == 0)
      {
      std::cout << "processed " << numberOfPointsProcessed << " points." << std::endl;
      }
    //std::cout << "Computing descriptor for pixel " << imageIterator.GetIndex() << std::endl;
    //fout << "Computing descriptor for pixel " << imageIterator.GetIndex() << std::endl;
    itk::ImageRegion<2> patchRegion = Helpers::GetRegionInRadiusAroundPixel(imageIterator.GetIndex(), patch_half_width);
    //std::cout << "patchRegion: " << patchRegion << std::endl;
    // Skip regions that are not entirely inside the image.
    if(!fullRegion.IsInside(patchRegion))
      {
      ++imageIterator;
      continue;
      }

    // Get a list of the pointIds in the region
    //std::vector<unsigned int> pointIds;
    std::vector<int> pointIds;
    // We can only add pointIds if the point corresponding to the pixel actually exists!
    for(unsigned int offsetId = 0; offsetId < validOffsets.size(); ++offsetId)
    {
      itk::Index<2> pixel = patchRegion.GetIndex() + validOffsets[offsetId];
      CoordinateMapType::iterator iter = this->CoordinateMap.find(pixel);
      if(iter != this->CoordinateMap.end())
        {
        // output the value that "testone" maps to
        //std::cout << "Found!" << std::endl;
        pointIds.push_back(iter->second);
        }
      else
        {
        //std::cout << "Not found!" << std::endl;
        }
      //pointIds.push_back(coordinateMap[pixel]);
    }

    if(pointIds.size() < 2)
      {
      unsigned int currentPointId = this->CoordinateMap[imageIterator.GetIndex()];

      featureCloud->points[currentPointId] = emptyPoint;
      ++imageIterator;
      continue;
      }
    //std::cout << "There are " << pointIds.size() << " points in this patch." << std::endl;

    // Setup the feature computation
    pcl::VFHEstimation<InputCloudType::PointType, pcl::Normal, OutputCloudType::PointType> vfhEstimation;

    //vfhEstimation.setIndices(&pointIds);
    vfhEstimation.setIndices(boost::make_shared<std::vector<int> >(pointIds));

    // Provide the original point cloud (without normals)
    vfhEstimation.setInputCloud (this->PCLCloud);

    // Provide the point cloud with normals
    vfhEstimation.setInputNormals(this->PCLCloudWithNormals);

    // vfhEstimation.setInputWithNormals(cloud, cloudWithNormals); VFHEstimation does not have this function
    // Use the same KdTree from the normal estimation
    vfhEstimation.setSearchMethod (this->NormalComputer.Tree);

    //vfhEstimation.setRadiusSearch (0.2);

    // Actually compute the VFH for this subset of points
    OutputCloudType::Ptr vfhFeature(new OutputCloudType);
    vfhEstimation.compute (*vfhFeature);

    unsigned int currentPointId = this->CoordinateMap[imageIterator.GetIndex()];

    featureCloud->points[currentPointId] = vfhFeature->points[0];

    ++imageIterator;
    }

  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName("Descriptors");
  descriptors->SetNumberOfComponents(308);
  descriptors->SetNumberOfTuples(this->PointCloud->GetNumberOfPoints());

//   // Zero all of the descriptors, we may not have one to assign for every point.
//   std::vector<float> zeroVector(308, 0);
// 
//   for(size_t pointId = 0; pointId < outputCloud->points.size(); ++pointId)
//     {
//     descriptors->SetTupleValue(pointId, zeroVector.data());
//     }

  for(vtkIdType pointId = 0; pointId < this->PointCloud->GetNumberOfPoints(); ++pointId)
    {
    descriptors->SetTupleValue(pointId, featureCloud->points[pointId].histogram);
    }

  this->PointCloud->GetPointData()->AddArray(descriptors);
}

void ComputeAndCompareDescriptorsWidget::ComputeDifferences()
{
  ComputeFeatures();

  vtkIdType numberOfPoints = this->PointCloud->GetNumberOfPoints();
  std::cout << "There are " << numberOfPoints << " points." << std::endl;

  vtkIdType selectedPointId = this->SelectionStyle->SelectedPointId;
  std::cout << "selectedPointId: " << selectedPointId << std::endl;

  if(selectedPointId < 0 || selectedPointId >= numberOfPoints)
    {
    std::cerr << "You must select a point to compare!" << std::endl;
    return;
    }

  std::string nameOfArrayToCompare = "Descriptors";

  vtkDataArray* descriptorArray = this->PointCloud->GetPointData()->GetArray(nameOfArrayToCompare.c_str());

  if(!descriptorArray)
    {
    throw std::runtime_error("Array not found!"); // The array should always be found because we are selecting it from a list of available arrays!
    }

  std::cout << "There are " << descriptorArray->GetNumberOfComponents() << " components in the descriptor." << std::endl;
  double* selectedDescriptor = new double[descriptorArray->GetNumberOfComponents()]; // This must be double because it is VTK's internal storage type
  descriptorArray->GetTuple(selectedPointId, selectedDescriptor);
  //std::cout << "selectedDescriptor: " << selectedDescriptor[0] << " " << selectedDescriptor[1] << std::endl;

  vtkSmartPointer<vtkFloatArray> differences = vtkSmartPointer<vtkFloatArray>::New();
  std::string descriptorDifferenceName = nameOfArrayToCompare + "_Differences";
  differences->SetName(descriptorDifferenceName.c_str());
  differences->SetNumberOfComponents(1);
  differences->SetNumberOfTuples(numberOfPoints);

  for(vtkIdType pointId = 0; pointId < this->PointCloud->GetNumberOfPoints(); ++pointId)
    {
    double* currentDescriptor = descriptorArray->GetTuple(pointId);
    //std::cout << "descriptor " << pointId << " : " << currentDescriptor[0] << " " << currentDescriptor[1] << std::endl;
    float difference = Helpers::ArrayDifference(selectedDescriptor, currentDescriptor, descriptorArray->GetNumberOfComponents());
    differences->SetValue(pointId, difference);
    }

  this->PointCloud->GetPointData()->AddArray(differences);
  this->PointCloud->GetPointData()->SetActiveScalars(descriptorDifferenceName.c_str());

  float range[2];
  differences->GetValueRange(range);
  vtkSmartPointer<vtkLookupTable> lookupTable = vtkSmartPointer<vtkLookupTable>::New();
  std::cout << "Range: " << range[0] << ", " << range[1] << std::endl;
  lookupTable->SetTableRange(range[0], range[1]);
  //lookupTable->SetHueRange(0, 1); // Don't do this, because 0 and 1 are the same in the H space of HSV!
  lookupTable->SetHueRange(0, .5);

  this->PointCloudMapper->SetLookupTable(lookupTable);
  //std::cout << "UseLookupTableScalarRange " << this->Pane->PointCloudMapper->GetUseLookupTableScalarRange() << std::endl;
  //this->Pane->PointCloudMapper->SetUseLookupTableScalarRange(false);

  // Without this, only a small band of colors is produce around the point.
  // I'm not sure why the scalar range of the data set is not the same?
  this->PointCloudMapper->SetUseLookupTableScalarRange(true);

  //this->qvtkWidget->GetRenderWindow()->Render(); // potentially causing threading related crash?

}

void ComputeAndCompareDescriptorsWidget::on_actionSave_activated()
{
  // Get a filename to save
  QString fileName = QFileDialog::getSaveFileName(this, "Save File", ".", "Point Clouds (*.vtp)");

  std::cout << "Got filename: " << fileName.toStdString() << std::endl;
  if(fileName.toStdString().empty())
    {
    std::cout << "Filename was empty." << std::endl;
    return;
    }

  SavePointCloud(fileName.toStdString());
}

void ComputeAndCompareDescriptorsWidget::SavePointCloud(const std::string& fileName)
{
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(fileName.c_str());
  writer->SetInputConnection(this->PointCloud->GetProducerPort());
  writer->Write();
}

void ComputeAndCompareDescriptorsWidget::on_actionOpenPointCloud_activated()
{
  // Get a filename to open
  QString fileName = QFileDialog::getOpenFileName(this, "Open File", ".", "Point Clouds (*.vtp)");

  std::cout << "Got filename: " << fileName.toStdString() << std::endl;
  if(fileName.toStdString().empty())
    {
    std::cout << "Filename was empty." << std::endl;
    return;
    }

  LoadPointCloud(fileName.toStdString());
}
