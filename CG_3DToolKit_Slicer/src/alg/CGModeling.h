#ifndef CGMODELING_H
#define CGMODELING_H

#pragma once

#include <stdio.h>
#include <cmath>
#include <string>
#include <vector>
#include <CGVTKHeader.h>

using namespace std;

namespace CG
{
    void LoadSTPFile(const string filename, vtkActor *actor);

    int LoadPWDFile(const std::string filename, std::vector<CG_Triangle> &triangles, std::vector<std::pair<std::size_t, double>> &minmax);

    int SavePWDFile(const std::string filename, std::vector<CG_Triangle> &triangles, std::vector<std::pair<std::size_t, double>> &minmax);

    CG_Vector_3 Normal(CG_Point_3 V1, CG_Point_3 V2, CG_Point_3 V3);

    CG_Point_3 TriangleCentroid(CG_Triangle tri);

    void TriangleMesh(std::vector<CG_Triangle> &triangles, vtkActor *actor);

    void TrianglePoints(std::vector<CG_Triangle> &triangles, vtkActor *actor);
}

inline unsigned char FloatToChar(float value)
{
    int intValue = std::floor(value * 255.0f + 0.5f);

    if (intValue < 0) intValue = 0;
    if (intValue > 255) intValue = 255;

    return static_cast<unsigned char>(intValue);
}

#endif // CGMODELING_H
