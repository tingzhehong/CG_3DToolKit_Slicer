#include "CGModeling.h"
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>

namespace CG
{

void ParseTopoDSShapeToOff(const TopoDS_Shape& aShape, std::vector<gp_Pnt> &vertexArray)
{
    //计算顶点和顶点索引//
    vertexArray.clear();
    BRepMesh_IncrementalMesh(aShape, 0.001);

    gp_Pnt vertex1;
    gp_Pnt vertex2;
    gp_Pnt vertex3;

    Standard_Integer nVertexIndex1 = 0;
    Standard_Integer nVertexIndex2 = 0;
    Standard_Integer nVertexIndex3 = 0;

    TopExp_Explorer faceExplorer;
    for (faceExplorer.Init(aShape, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next())
    {
        TopLoc_Location loc;
        TopoDS_Face aFace = TopoDS::Face(faceExplorer.Current());
        Handle_Poly_Triangulation triFace = BRep_Tool::Triangulation(aFace, loc);
        if (triFace.IsNull())
        {
            continue;
        }

        Standard_Boolean hasNormal = triFace->HasNormals();
        Standard_Boolean hasuvNormal = triFace->HasUVNodes();
        Standard_Integer l = triFace->Nodes().Length();
        Standard_Integer nTriangles = triFace->NbTriangles();

        TColgp_Array1OfPnt nodes(1, triFace->NbNodes());
        Poly_Array1OfTriangle triangles(1, triFace->NbTriangles());
        nodes = triFace->Nodes();
        triangles = triFace->Triangles();

        for (Standard_Integer i = 1; i <= nTriangles; i++)
        {
            Poly_Triangle aTriangle = triangles.Value(i);
            aTriangle.Get(nVertexIndex1, nVertexIndex2, nVertexIndex3);

            vertex1 = nodes.Value(nVertexIndex1);
            vertex2 = nodes.Value(nVertexIndex2);
            vertex3 = nodes.Value(nVertexIndex3);

            vertexArray.push_back(gp_Pnt(vertex1.X(), vertex1.Y(), vertex1.Z()));
            vertexArray.push_back(gp_Pnt(vertex2.X(), vertex2.Y(), vertex2.Z()));
            vertexArray.push_back(gp_Pnt(vertex3.X(), vertex3.Y(), vertex3.Z()));
        }
    }
}

void LoadSTPFile(const string filename, vtkActor *actor)
{
    Standard_CString STEPfile = filename.c_str();

    STEPControl_Reader aReader_Step;
    aReader_Step.ReadFile(STEPfile); //读取STEP文件
    aReader_Step.PrintCheckLoad(Standard_False, IFSelect_ItemsByEntity); //检查文件加载状态

    Standard_Integer NbRoots = aReader_Step.NbRootsForTransfer();// 获取可转移根的数量
    Standard_Integer num = aReader_Step.TransferRoots();//翻译所有可转换的根，并返回//成功翻译的次数

    TopoDS_Shape aShape = aReader_Step.OneShape();//读取到TopoDS_Shape结构中

    //获取模型顶点
    std::vector<gp_Pnt> aVertexArray;
    ParseTopoDSShapeToOff(aShape, aVertexArray);

    // 构建mesh
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType index = 0;

    for (size_t i = 0; i < aVertexArray.size() - 3; i += 3)
    {
        gp_Pnt pt1 = aVertexArray[i + 0];
        gp_Pnt pt2 = aVertexArray[i + 1];
        gp_Pnt pt3 = aVertexArray[i + 2];

        // 创建点数据
        points->InsertNextPoint(pt1.X(), pt1.Y(), pt1.Z());
        points->InsertNextPoint(pt2.X(), pt2.Y(), pt2.Z());
        points->InsertNextPoint(pt3.X(), pt3.Y(), pt3.Z());

        // 创建多边形数据
        vtkIdType polygon[3] = { index + 0, index + 1, index + 2 };
        polygons->InsertNextCell(3, polygon);

        index += 3;
    }

    // 创建PolyData对象并设置点和多边形数据
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetPolys(polygons);
    g_PolyData = polyData;

    // 创建Mapper
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    // 将Mapper数据交给actor
    actor->SetMapper(mapper);
}

}
