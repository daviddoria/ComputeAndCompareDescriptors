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
class vtkDataSetMapper;
class vtkRenderer;
class vtkStructuredGrid;

/* 
 * This class will compute a descriptor at each point (if it isn't already computed, indicated by the existance of
 * an array with the same name), and then compute the differences of the descriptor at the currently selected point
 * with the other points, and color the result by this difference.
 */
class ComputeAndCompareDescriptorsWidget : public QMainWindow, public Ui::ComputeAndCompareDescriptorsWidget
{
  Q_OBJECT
public:

  typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudType;
  // This overkill for most descriptors, but some need the RGB. That is, this type is a superset of any requirement of the feature filters.
  typedef pcl::PointCloud<pcl::PointXYZRGBNormal> FullCloudType; 

  typedef itk::Image<bool, 2> MaskImageType;

  // Constructor/Destructor
  ComputeAndCompareDescriptorsWidget();

  ~ComputeAndCompareDescriptorsWidget() {};

  void LoadData(const std::string& fileName);
  void LoadPolyData(const std::string& fileName);
  void LoadStructuredGrid(const std::string& fileName);
  void LoadMask(const std::string& fileName);

public slots:
  void on_actionOpenPointCloud_activated();
  void on_actionSave_activated();

  void on_btnCompare_clicked();

  void on_actionHelp_activated();
  void on_actionQuit_activated();

private:

//   template <typename T>
//   void RegisterDescriptorComputer(T);
  void RegisterDescriptorComputer(const std::string& descriptorName);
  
  void ComputeDifferences();
  void ComputeFeatures();
  void CreateIndexMap();

  typedef std::map<itk::Index<2>, unsigned int, itk::Index<2>::LexicographicCompare> CoordinateMapType;
  CoordinateMapType CoordinateMap;

  void SavePointCloud(const std::string& fileName);

  vtkSmartPointer<vtkPolyData> PolyData;
  vtkSmartPointer<vtkStructuredGrid> StructuredGrid;

  void SharedConstructor();
  QFutureWatcher<void> FutureWatcher;
  QProgressDialog* ProgressDialog;

  vtkSmartPointer<vtkPointPicker> PointPicker;

  vtkSmartPointer<vtkRenderer> Renderer;

  vtkSmartPointer<PointSelectionStyle3D> SelectionStyle;

  void Refresh();

  vtkSmartPointer<vtkActor> PointCloudActor;
  vtkSmartPointer<vtkDataSetMapper> PointCloudMapper;

  vtkSmartPointer<vtkActor> MarkerActor;
  vtkSmartPointer<vtkPolyDataMapper> MarkerMapper;
  vtkSmartPointer<vtkSphereSource> MarkerSource;

  float MarkerRadius;

  void SelectedPointCallback(vtkObject* caller, long unsigned int eventId, void* callData);

  FullCloudType::Ptr PCLCloud;

  MaskImageType::Pointer Mask;

  ComputeNormals<FullCloudType::PointType, FullCloudType::PointType> NormalComputer;
  
  enum InputCloudTypeEnum { VTP, VTS};
  InputCloudTypeEnum InputCloudType;
};

#include "ComputeAndCompareDescriptorsWidget.hpp"

#endif
