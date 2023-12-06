﻿#ifndef CGVTKHEADER_H
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

#endif // CGVTKHEADER_H
