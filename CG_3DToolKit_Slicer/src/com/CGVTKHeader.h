#ifndef CGVTKHEADER_H
#define CGVTKHEADER_H

#pragma once

// VTK
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyVertex.h>
#include <vtkDoubleArray.h>
#include <vtkUnstructuredGrid.h>
#include <vtkPolyVertex.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkPlane.h>
#include <vtkPlanes.h>
#include <vtkActor.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkElevationFilter.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkOutlineFilter.h>
#include <vtkProperty.h>
#include <vtkLookupTable.h>
#include <vtkDataSetMapper.h>
#include <vtkScalarBarActor.h>
#include <vtkScalarBarWidget.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkAutoInit.h>
#include <vtkPLYReader.h>
#include <vtkSTLReader.h>
#include <vtkOBJReader.h>

VTK_MODULE_INIT(vtkRenderingOpenGL)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingFreeType)

extern vtkSmartPointer<vtkActor> g_Actor;
extern vtkSmartPointer<vtkPolyData> g_PolyData;

struct CG_Point_3
{
    float x;
    float y;
    float z;

    CG_Point_3()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    CG_Point_3(double X, double Y, double Z)
    {
        x = X;
        y = Y;
        z = Z;
    }
};

struct CG_Vector_3
{
    double x;
    double y;
    double z;

    CG_Vector_3()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    CG_Vector_3(double X, double Y, double Z)
    {
        x = X;
        y = Y;
        z = Z;
    }
};

struct CG_Triangle
{
    CG_Point_3 vertx1;
    CG_Point_3 vertx2;
    CG_Point_3 vertx3;
    CG_Point_3 centroid;

    float r;
    float g;
    float b;
    float d;

    CG_Triangle()
    {
        r = 1.0;
        g = 1.0;
        b = 1.0;
        d = 0.0;
    }
};

#endif // CGVTKHEADER_H
