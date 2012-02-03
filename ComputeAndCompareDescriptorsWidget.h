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

#ifndef VFHComparisonWidget_H
#define VFHComparisonWidget_H

#include "ui_ComputeAndCompareDescriptorsWidget.h"

// VTK
#include <vtkSmartPointer.h>
#include <vtkSeedWidget.h>

// ITK
#include "itkImage.h"

// Qt
#include <QMainWindow>
#include <QFutureWatcher>
class QProgressDialog;

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Custom
#include "PointSelectionStyle3D.h"
#include "Types.h"
#include "ComputeNormals.h"

// Forward declarations
class vtkActor;
class vtkBorderWidget;
class vtkImageData;
class vtkImageActor;
class vtkPointPicker;
class vtkPolyData;
class vtkPolyDataMapper;
class vtkRenderer;

class ComputeAndCompareDescriptorsWidget : public QMainWindow, public Ui::ComputeAndCompareDescriptorsWidget
{
  Q_OBJECT
public:

  enum DescriptorEnum {CVFH, VFH, MASKED_VFH, SPINIMAGES, SHAPECONTEXTS, FPFH, PFH};

  typedef pcl::PointCloud<pcl::PointXYZ> InputCloudType;

  typedef pcl::PointCloud<pcl::Normal> NormalsCloudType;

  typedef itk::Image<bool, 2> MaskImageType;

  // Constructor/Destructor
  ComputeAndCompareDescriptorsWidget();

  ~ComputeAndCompareDescriptorsWidget() {};

  void LoadPointCloud(const std::string& fileName);
  void LoadMask(const std::string& fileName);

public slots:
  void on_actionOpenPointCloud_activated();
  void on_actionSave_activated();

  void on_btnCompute_clicked();

  void on_actionHelp_activated();
  void on_actionQuit_activated();

private:

  void ComputeDifferences();
  void ComputeFeatures();
  void CreateIndexMap();

  typedef std::map<itk::Index<2>, unsigned int, itk::Index<2>::LexicographicCompare> CoordinateMapType;
  CoordinateMapType CoordinateMap;

  void SavePointCloud(const std::string& fileName);

  vtkSmartPointer<vtkPolyData> PointCloud;

  void SharedConstructor();
  QFutureWatcher<void> FutureWatcher;
  QProgressDialog* ProgressDialog;

  vtkSmartPointer<vtkPointPicker> PointPicker;

  vtkSmartPointer<vtkRenderer> Renderer;

  vtkSmartPointer<PointSelectionStyle3D> SelectionStyle;

  void Refresh();

  vtkSmartPointer<vtkActor> PointCloudActor;
  vtkSmartPointer<vtkPolyDataMapper> PointCloudMapper;

  vtkSmartPointer<vtkActor> MarkerActor;
  vtkSmartPointer<vtkPolyDataMapper> MarkerMapper;
  vtkSmartPointer<vtkSphereSource> MarkerSource;

  float MarkerRadius;

  void SelectedPointCallback(vtkObject* caller, long unsigned int eventId, void* callData);

  InputCloudType::Ptr PCLCloud;
  NormalsCloudType::Ptr PCLCloudWithNormals;

  MaskImageType::Pointer Mask;

  ComputeNormals NormalComputer;
};

#endif
