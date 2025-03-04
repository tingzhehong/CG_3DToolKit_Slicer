OCCTPATH = "D:/Program Files/OpenCASCADE-7.5.0-vc14-64/opencascade-7.5.0"

INCLUDEPATH += $$quote($${OCCTPATH}/inc)

LIBS += -L$$quote($${OCCTPATH}/win64/vc14/lib)

LIBS +=           \
    -lTKBin       \
    -lTKBinL      \
    -lTKBinTObj   \
    -lTKBinXCAF   \
    -lTKBO        \
    -lTKBool      \
    -lTKBRep      \
    -lTKCAF       \
    -lTKCDF       \
    -lTKD3DHost   \
    -lTKDCAF      \
    -lTKDFBrowser \
    -lTKDraw      \
    -lTKernel     \
    -lTKFeat      \
    -lTKFillet    \
    -lTKG2d       \
    -lTKG3d       \
    -lTKGeomAlgo  \
    -lTKGeomBase  \
    -lTKHLR       \
    -lTKIGES      \
    -lTKIVtk      \
    -lTKIVtkDraw  \
    -lTKLCAF      \
    -lTKMath      \
    -lTKMesh      \
    -lTKMeshVS    \
    -lTKOffset    \
    -lTKOpenGl    \
    -lTKPrim      \
    -lTKQADraw    \
    -lTKService   \
    -lTKShapeView \
    -lTKShHealing \
    -lTKStd       \
    -lTKStdL      \
    -lTKSTEP      \
    -lTKSTEP209   \
    -lTKSTEPAttr  \
    -lTKSTEPBase  \
    -lTKSTL       \
    -lTKTInspector    \
    -lTKTInspectorAPI \
    -lTKTObj      \
    -lTKTObjDRAW  \
    -lTKToolsDraw \
    -lTKTopAlgo   \
    -lTKTopTest   \
    -lTKTreeModel \
    -lTKV3d       \
    -lTKVCAF      \
    -lTKView      \
    -lTKViewerTest\
    -lTKVInspector\
    -lTKVRML      \
    -lTKXCAF      \
    -lTKXDEDRAW   \
    -lTKXDEIGES   \
    -lTKXDESTEP   \
    -lTKXMesh     \
    -lTKXml       \
    -lTKXmlL      \
    -lTKXmlTObj   \
    -lTKXmlXCAF   \
    -lTKXSBase    \
    -lTKXSDRAW
