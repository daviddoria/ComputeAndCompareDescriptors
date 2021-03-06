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

#include "ComputeAndCompareDescriptorsWidget.h"

// ITK
#include "itkCastImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkRegionOfInterestImageFilter.h"

// PCL
#include <pcl/common/io.h> // For copyPointCloud

// Qt
#include <QFileDialog>
#include <QIcon>
#include <QProgressDialog>
#include <QTextEdit>
#include <QtConcurrentRun>
#include <QMessageBox>

// VTK
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkDataSetMapper.h>
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
#include <vtkXMLStructuredGridReader.h>
#include <vtkXMLStructuredGridWriter.h>

// Boost
#include <boost/make_shared.hpp>

// Custom
#include "Helpers.h"
#include "Types.h"
#include "PointSelectionStyle3D.h"
#include "VTKtoPCL.h"
#include "ComputeNormals.h"
#include "ComputeClusteredViewpointFeatureHistograms.h"
#include "ComputeViewpointFeatureHistograms.h"
#include "ComputePointFeatureHistograms.h"
#include "ComputeFastPointFeatureHistograms.h"
#include "ComputePFHRGB.h"

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
MarkerRadius(.05), PCLCloud(new FullCloudType)
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

  // Point data
  this->PolyData = vtkSmartPointer<vtkPolyData>::New();
  this->StructuredGrid = vtkSmartPointer<vtkStructuredGrid>::New();

  this->PointCloudMapper = vtkSmartPointer<vtkDataSetMapper>::New();

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
  
  //cmbDescriptor->addItem("CVFH");
  RegisterDescriptorComputer(ComputeClusteredViewpointFeatureHistograms::DescriptorName);
  RegisterDescriptorComputer(ComputeViewpointFeatureHistograms::DescriptorName);
  RegisterDescriptorComputer(ComputePointFeatureHistograms::DescriptorName);
  RegisterDescriptorComputer(ComputeFastPointFeatureHistograms::DescriptorName);
  RegisterDescriptorComputer(ComputePFHRGB::DescriptorName);
}

void ComputeAndCompareDescriptorsWidget::SelectedPointCallback(vtkObject* caller, long unsigned int eventId, void* callData)
{
  double p[3];
  
  if(this->InputCloudType == VTP)
    {
    this->PolyData->GetPoint(this->SelectionStyle->SelectedPointId, p);
    }
  else if(this->InputCloudType == VTS)
    {
    this->StructuredGrid->GetPoint(this->SelectionStyle->SelectedPointId, p);
    }

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

void ComputeAndCompareDescriptorsWidget::RegisterDescriptorComputer(const std::string& descriptorName)
{
  this->cmbDescriptor->addItem(descriptorName.c_str());
}

void ComputeAndCompareDescriptorsWidget::LoadPolyData(const std::string& fileName)
{
  InputCloudType = VTP;

  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(fileName.c_str());
  reader->Update();

  // Keep all points
  //this->PointCloud->DeepCopy(reader->GetOutput());

  // Keep only unmasked points
  std::cout << "Displaying only unmasked points..." << std::endl;
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
  this->PolyData->DeepCopy(surfaceFilter->GetOutput());

  this->PointCloudActor->GetProperty()->SetRepresentationToPoints();

  this->Renderer->ResetCamera();

  this->PointPicker->AddPickList(this->PointCloudActor);

  this->qvtkWidget->GetRenderWindow()->GetInteractor()->SetPicker(this->PointPicker);
  this->SelectionStyle->SetPoints(this->PolyData->GetPoints());
  this->SelectionStyle->SetCurrentRenderer(this->Renderer);
  this->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(this->SelectionStyle);

  this->Renderer->ResetCamera();

  std::cout << "Converting to PCL..." << std::endl;

  VTKtoPCL(this->PolyData.GetPointer(), this->PCLCloud.get());

  std::cout << "Computing normals..." << std::endl;
  this->NormalComputer(this->PCLCloud, this->PCLCloud);

  std::cout << "LoadPolyData(): Creating index map..." << std::endl;
  CreateIndexMap();

  // Display the polydata
  this->PointCloudMapper->SetInputConnection(this->PolyData->GetProducerPort());
  this->SelectionStyle->GetCurrentRenderer()->ResetCamera();
  this->qvtkWidget->GetRenderWindow()->Render();
  std::cout << "Finished LoadPolyData()..." << std::endl;
}

void ComputeAndCompareDescriptorsWidget::LoadStructuredGrid(const std::string& fileName)
{
  InputCloudType = VTS;

  vtkSmartPointer<vtkXMLStructuredGridReader> reader = vtkSmartPointer<vtkXMLStructuredGridReader>::New();
  reader->SetFileName(fileName.c_str());
  reader->Update();

  this->StructuredGrid->DeepCopy(reader->GetOutput());

  this->PointCloudMapper->SetInputConnection(this->PolyData->GetProducerPort());

  this->PointCloudActor->GetProperty()->SetRepresentationToPoints();

  this->Renderer->ResetCamera();

  this->PointPicker->AddPickList(this->PointCloudActor);

  this->qvtkWidget->GetRenderWindow()->GetInteractor()->SetPicker(this->PointPicker);
  this->SelectionStyle->SetPoints(this->StructuredGrid->GetPoints());
  this->SelectionStyle->SetCurrentRenderer(this->Renderer);
  this->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(this->SelectionStyle);

  this->Renderer->ResetCamera();

  std::cout << "Converting to PCL..." << std::endl;

  VTKtoPCL(this->StructuredGrid.GetPointer(), this->PCLCloud.get());

  std::cout << "Computing normals..." << std::endl;
  this->NormalComputer(this->PCLCloud, this->PCLCloud);

  // Display the polydata
  this->PointCloudMapper->SetInputConnection(this->StructuredGrid->GetProducerPort());
  this->SelectionStyle->GetCurrentRenderer()->ResetCamera();
  this->qvtkWidget->GetRenderWindow()->Render();
  std::cout << "Finished LoadStructuredGrid()..." << std::endl;
}

void ComputeAndCompareDescriptorsWidget::on_btnCompare_clicked()
{
  // If features already exist
  bool featuresAlreadyExist = false;
  if(this->InputCloudType == VTP)
    {
    if(this->PolyData->GetPointData()->HasArray(cmbDescriptor->currentText().toStdString().c_str()))
      {
      featuresAlreadyExist = true;
      }
    }
  else if(this->InputCloudType == VTS)
    {
    if(this->StructuredGrid->GetPointData()->HasArray(cmbDescriptor->currentText().toStdString().c_str()))
      {
      featuresAlreadyExist = true;
      }
    }

  if(featuresAlreadyExist)
    {
    QMessageBox msgBox;
    msgBox.addButton(QMessageBox::Yes);
    msgBox.addButton(QMessageBox::No);
    msgBox.setText("This feature already exists. Do you want to recompute the features?");

    int selection = msgBox.exec();
    if(selection == QMessageBox::Yes)
      {
      QFuture<void> future = QtConcurrent::run(this, &ComputeAndCompareDescriptorsWidget::ComputeFeatures);
      this->FutureWatcher.setFuture(future);
      this->ProgressDialog->setMinimum(0);
      this->ProgressDialog->setMaximum(0);
      this->ProgressDialog->setLabelText("Computing features...");
      this->ProgressDialog->setWindowModality(Qt::WindowModal);
      this->ProgressDialog->exec();
      }
    }

  ComputeDifferences();
}

void ComputeAndCompareDescriptorsWidget::CreateIndexMap()
{
  std::cout << "Creating index map..." << std::endl;

  // Add a map from all of the pixels to their corresponding point id. 
  vtkIntArray* indexArray = vtkIntArray::SafeDownCast(this->PolyData->GetPointData()->GetArray("OriginalPixel"));
  for(vtkIdType pointId = 0; pointId < this->PolyData->GetNumberOfPoints(); ++pointId)
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
  std::cout << "ComputeFeatures()..." << std::endl;

  if(cmbDescriptor->currentText().toStdString() == "CVFH")
    {
    ComputeClusteredViewpointFeatureHistograms cvfhComputer;
    PointNormalCloudType::Ptr cloud ( new PointNormalCloudType);
    copyPointCloud(*this->PCLCloud, *cloud);
    cvfhComputer(cloud, this->Mask, this->PolyData);
    }
  else if(cmbDescriptor->currentText().toStdString() == "VFH")
    {
    ComputeViewpointFeatureHistograms vfhComputer;
    ComputeClusteredViewpointFeatureHistograms cvfhComputer;
    PointNormalCloudType::Ptr cloud ( new PointNormalCloudType);
    copyPointCloud(*this->PCLCloud, *cloud);
    //vfhComputer(cloud, this->Mask.GetPointer(), this->PolyData.GetPointer());
    vfhComputer(cloud, this->Mask.GetPointer(), this->StructuredGrid.GetPointer());
    }
  else if(cmbDescriptor->currentText().toStdString() == "PFH")
    {
    ComputePointFeatureHistograms pfhComputer;
    ComputeClusteredViewpointFeatureHistograms cvfhComputer;
    PointNormalCloudType::Ptr cloud ( new PointNormalCloudType);
    copyPointCloud(*this->PCLCloud, *cloud);
    pfhComputer(cloud, this->PolyData);
    }
  else if(cmbDescriptor->currentText().toStdString() == "FPFH")
    {
    ComputeFastPointFeatureHistograms fpfhComputer;
    ComputeClusteredViewpointFeatureHistograms cvfhComputer;
    PointNormalCloudType::Ptr cloud ( new PointNormalCloudType);
    copyPointCloud(*this->PCLCloud, *cloud);
    fpfhComputer(cloud, this->PolyData);
    }
  else if(cmbDescriptor->currentText().toStdString() == "PFHRGB")
    {
    ComputePFHRGB pfhrgbComputer;
    pfhrgbComputer(this->PCLCloud, this->Mask, this->PolyData);
    }
  else
    {
    throw std::runtime_error("No known method for computing " + cmbDescriptor->currentText().toStdString() + " descriptors!");
    }
}

void ComputeAndCompareDescriptorsWidget::ComputeDifferences()
{
  std::cout << "ComputeDifferences()..." << std::endl;

  vtkIdType numberOfPoints = 0;
  if(this->InputCloudType == VTP)
  {
    numberOfPoints = this->PolyData->GetNumberOfPoints();
  }
  else if(this->InputCloudType == VTS)
  {
    numberOfPoints = this->StructuredGrid->GetNumberOfPoints();
  }

  //std::cout << "There are " << numberOfPoints << " points." << std::endl;

  vtkIdType selectedPointId = this->SelectionStyle->SelectedPointId;
  //std::cout << "selectedPointId: " << selectedPointId << std::endl;

  if(selectedPointId < 0 || selectedPointId >= numberOfPoints)
    {
    std::cerr << "You must select a point to compare! (selectedPointId is " << selectedPointId << " and numberOfPoints is " << numberOfPoints << ")" << std::endl;
    return;
    }

  std::string nameOfArrayToCompare = cmbDescriptor->currentText().toStdString();

  vtkDataArray* descriptorArray;
  if(this->InputCloudType == VTP)
    {
    descriptorArray = this->PolyData->GetPointData()->GetArray(nameOfArrayToCompare.c_str());
    }
  else if(this->InputCloudType == VTS)
    {
    descriptorArray = this->StructuredGrid->GetPointData()->GetArray(nameOfArrayToCompare.c_str());
    }

  if(!descriptorArray)
    {
    throw std::runtime_error("Array " + nameOfArrayToCompare + " not found!"); // The array should always be found because we are selecting it from a list of available arrays!
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

  for(vtkIdType pointId = 0; pointId < this->PolyData->GetNumberOfPoints(); ++pointId)
    {
    double* currentDescriptor = descriptorArray->GetTuple(pointId);
    //std::cout << "descriptor " << pointId << " : " << currentDescriptor[0] << " " << currentDescriptor[1] << std::endl;
    float difference = Helpers::ArrayDifference(selectedDescriptor, currentDescriptor, descriptorArray->GetNumberOfComponents());
    differences->SetValue(pointId, difference);
    }

  if(this->InputCloudType == VTP)
    {
    this->PolyData->GetPointData()->AddArray(differences);
    this->PolyData->GetPointData()->SetActiveScalars(descriptorDifferenceName.c_str());
    }
  else if(this->InputCloudType == VTS)
    {
    this->StructuredGrid->GetPointData()->AddArray(differences);
    this->StructuredGrid->GetPointData()->SetActiveScalars(descriptorDifferenceName.c_str());
    }

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

  //this->qvtkWidget->GetRenderWindow()->Render(); // Can't do this here because we are potentially currently in a different thread from the renderer (if we are using a progress bar + future watcher)

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
  writer->SetInputConnection(this->PolyData->GetProducerPort());
  writer->Write();
}

void ComputeAndCompareDescriptorsWidget::LoadData(const std::string& fileName)
{
  QFileInfo fileInfo(fileName.c_str());
  if(fileInfo.suffix().toStdString() == "vtp")
  {
    LoadPolyData(fileName);
  }
  else if(fileInfo.suffix().toStdString() == "vts")
  {
    LoadStructuredGrid(fileName);
  }
}

void ComputeAndCompareDescriptorsWidget::on_actionOpenPointCloud_activated()
{
  // Get a filename to open
  QString fileName = QFileDialog::getOpenFileName(this, "Open File", ".", "Point Clouds (*.vtp;;*.vts)");

  std::cout << "Got filename: " << fileName.toStdString() << std::endl;
  if(fileName.toStdString().empty())
    {
    std::cout << "Filename was empty." << std::endl;
    return;
    }

  LoadData(fileName.toStdString());
}
