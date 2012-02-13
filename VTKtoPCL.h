#ifndef VTKtoPCL_H
#define VTKtoPCL_H

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkUnsignedCharArray.h>

template <typename VTKObjectT, typename PointT> 
void VTKtoPCL(VTKObjectT* const VTKObject, typename pcl::PointCloud<PointT>* const cloud)
{
  // This generic template will convert any VTK PolyData
  // to a coordinate-only PointXYZ PCL format.

  cloud->width = VTKObject->GetNumberOfPoints();
  cloud->height = 1; // This indicates that the point cloud is unorganized
  cloud->is_dense = false;
  cloud->points.resize(cloud->width);

  vtkUnsignedCharArray* colors = vtkUnsignedCharArray::SafeDownCast(VTKObject->GetPointData()->GetArray("Colors"));
  vtkFloatArray* normals = vtkFloatArray::SafeDownCast(VTKObject->GetPointData()->GetNormals());

  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
    // Coordinate
    double p[3];
    VTKObject->GetPoint(i,p);
    cloud->points[i].x = p[0];
    cloud->points[i].y = p[1];
    cloud->points[i].z = p[2];

    // Color
    if(colors)
      {
      unsigned char color[3];
      colors->GetTupleValue(i,color);
      cloud->points[i].r = color[0];
      cloud->points[i].g = color[1];
      cloud->points[i].b = color[2];
      }
      
    if(normals)
    {
      // Setup normals
      float normal[3];
      normals->GetTupleValue(i,normal);
      cloud->points[i].normal_x = normal[0];
      cloud->points[i].normal_y = normal[1];
      cloud->points[i].normal_z = normal[2];
    }
    }
}

/*
//Specialization for points with RGB values
template <>
void VTKtoPCL<pcl::PointXYZRGB> (vtkPolyData* polydata, CloudPointColorPtr cloud)
{
  vtkUnsignedCharArray* colors = 
    vtkUnsignedCharArray::SafeDownCast(polydata->GetPointData()->GetNormals());
    
  cloud->width = polydata->GetNumberOfPoints();
  cloud->height = 1; // This indicates that the point cloud is unorganized
  cloud->is_dense = false;
  cloud->points.resize(cloud->width);

  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
    // Setup points
    double p[3];
    polydata->GetPoint(i,p);
    cloud->points[i].x = p[0];
    cloud->points[i].y = p[1];
    cloud->points[i].z = p[2];
  
    // Setup colors
    unsigned char color[3];
    colors->GetTupleValue(i,color);
    cloud->points[i].r = color[0];
    cloud->points[i].g = color[1];
    cloud->points[i].b = color[2];
    }
}

template <> 
void VTKtoPCL<pcl::PointXYZRGBNormal> (vtkPolyData* polydata, CloudPointColorNormalPtr cloud)
{
  vtkFloatArray* normals = 
    vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetNormals());
    
  vtkUnsignedCharArray* colors = 
    vtkUnsignedCharArray::SafeDownCast(polydata->GetPointData()->GetNormals());
    
  cloud->width = polydata->GetNumberOfPoints();
  cloud->height = 1; // This indicates that the point cloud is unorganized
  cloud->is_dense = false;
  cloud->points.resize(cloud->width);

  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
    // Setup points
    double p[3];
    polydata->GetPoint(i,p);
    cloud->points[i].x = p[0];
    cloud->points[i].y = p[1];
    cloud->points[i].z = p[2];
  
    // Setup colors
    unsigned char color[3];
    colors->GetTupleValue(i,color);
    cloud->points[i].data_c[0] = color[0];
    cloud->points[i].data_c[1] = color[1];
    cloud->points[i].data_c[2] = color[2];
  
    // Setup normals
    float normal[3];
    normals->GetTupleValue(i,normal);
    cloud->points[i].normal_x = normal[0];
    cloud->points[i].normal_y = normal[1];
    cloud->points[i].normal_z = normal[2];
    }
}


template <> 
void VTKtoPCL<pcl::PointNormal> (vtkPolyData* polydata, CloudPointNormalPtr cloud)
{
  vtkFloatArray* normals = vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetNormals());
    
  cloud->width = polydata->GetNumberOfPoints();
  cloud->height = 1; // This indicates that the point cloud is unorganized
  cloud->is_dense = false;
  cloud->points.resize(cloud->width);

  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
    // Setup points
    double p[3];
    polydata->GetPoint(i,p);
    cloud->points[i].x = p[0];
    cloud->points[i].y = p[1];
    cloud->points[i].z = p[2];
  
    // Setup normals
    float normal[3];
    normals->GetTupleValue(i,normal);
    cloud->points[i].normal_x = normal[0];
    cloud->points[i].normal_y = normal[1];
    cloud->points[i].normal_z = normal[2];
    }
}*/

#endif
