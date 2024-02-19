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
}

#endif // CGMODELING_H
