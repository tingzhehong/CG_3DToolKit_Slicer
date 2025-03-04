#include "CGModeling.h"
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <Poly_Triangulation.hxx>
#include <BRep_Tool.hxx>

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

    Standard_Integer NbRoots = aReader_Step.NbRootsForTransfer(); // 获取可转移根的数量
    Standard_Integer num = aReader_Step.TransferRoots(); //翻译所有可转换的根，并返回//成功翻译的次数

    TopoDS_Shape aShape = aReader_Step.OneShape(); //读取到TopoDS_Shape结构中

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

int LoadPWDFile(const std::string filename, std::vector<CG_Triangle> &triangles, std::vector<std::pair<size_t, double> > &minmax)
{
    std::cout << "Read PWD File: " << filename << std::endl << std::endl;

    triangles.clear();

    ifstream inputFile(filename);
    if (!inputFile) return -1;
    string oneLine;
    int rowCount = 0;

    while (getline(inputFile, oneLine))
    {
        ++rowCount;
        if (rowCount < 7)
            continue;

        istringstream isOneLine(oneLine);
        string str;
        int colCount = 0;
        float v1_x, v1_y, v1_z, v2_x, v2_y, v2_z, v3_x, v3_y, v3_z;
        float r, g, b, d;

        if (rowCount == 7)
        {
            std::pair<std::size_t, double> value;
            while (getline(isOneLine, str, ','))
            {
                ++colCount;
                if (colCount % 2 == 1)
                {
                    value.first = static_cast<size_t>(atoi(str.c_str()));
                }
                if (colCount % 2 == 0)
                {
                    value.second = static_cast<double>(atof(str.c_str()));
                    minmax.push_back(value);
                }
            }
            colCount = 0;
        }

        if (rowCount < 9)
            continue;

        while (getline(isOneLine, str, ','))
        {
            ++colCount;
            if (colCount > 13)
                break;

            float f = atof(str.c_str());

            switch (colCount)
            {
            case 1:
                v1_x = f;
                break;
            case 2:
                v1_y = f;
                break;
            case 3:
                v1_z = f;
                break;
            case 4:
                v2_x = f;
                break;
            case 5:
                v2_y = f;
                break;
            case 6:
                v2_z = f;
                break;
            case 7:
                v3_x = f;
                break;
            case 8:
                v3_y = f;
                break;
            case 9:
                v3_z = f;
                break;
            case 10:
                r = f;
                break;
            case 11:
                g = f;
                break;
            case 12:
                b = f;
                break;
            case 13:
                d = f;
                break;
            }
        }

        //Triangle
        CG_Triangle myTriangle;
        myTriangle.vertx1 = CG_Point_3(v1_x, v1_y, v1_z);
        myTriangle.vertx2 = CG_Point_3(v2_x, v2_y, v2_z);
        myTriangle.vertx3 = CG_Point_3(v3_x, v3_y, v3_z);

        myTriangle.centroid = CG::TriangleCentroid(myTriangle);

        myTriangle.r = r;
        myTriangle.g = g;
        myTriangle.b = b;
        myTriangle.d = d;

        triangles.push_back(myTriangle);
    }
    inputFile.close();

    return 0;
}

int SavePWDFile(const std:: string filename, std::vector<CG_Triangle> &triangles, std::vector<std::pair<size_t, double> > &minmax)
{
    std::cout << "Write PWD File: " << filename << std::endl << std::endl;

    time_t NowTime;
    tm *pNowTime;
    time(&NowTime);
    pNowTime = localtime(&NowTime);

    std::string ThisTime = std::to_string(pNowTime->tm_year + 1900) + "-" + std::to_string(pNowTime->tm_mon + 1) + "-" + std::to_string(pNowTime->tm_mday) + "  " +
                           std::to_string(pNowTime->tm_hour) + ":" + std::to_string(pNowTime->tm_min) + ":" + std::to_string(pNowTime->tm_sec);

    ofstream outputFile(filename);

    outputFile << "Machine: CGMACHINE" << std::endl;
    outputFile << "Created By: CGSOFT//USER PolyWroks" << std::endl;
    outputFile << "Created On: " << ThisTime << std::endl;
    outputFile << "CGMesh: Triangle" << std::endl;
    outputFile << "Vertx,RGB,Distance" << std::endl;
    outputFile << std::endl;

    for (size_t j = 0; j < minmax.size(); ++j)
    {
        outputFile << minmax.at(j).first << "," << minmax.at(j).second << ",";
    }
    outputFile << std::endl << std::endl;

    for (size_t i = 0; i < triangles.size(); ++i)
    {
        double v1_x = triangles[i].vertx1.x; double v1_y = triangles[i].vertx1.y; double v1_z = triangles[i].vertx1.z;
        double v2_x = triangles[i].vertx2.x; double v2_y = triangles[i].vertx2.y; double v2_z = triangles[i].vertx2.z;
        double v3_x = triangles[i].vertx3.x; double v3_y = triangles[i].vertx3.y; double v3_z = triangles[i].vertx3.z;

        float r = triangles[i].r; float g = triangles[i].g; float b = triangles[i].b; float d = triangles[i].d;

        outputFile  << v1_x << "," << v1_y << "," << v1_z << ","
                    << v2_x << "," << v2_y << "," << v2_z << ","
                    << v3_x << "," << v3_y << "," << v3_z << ","
                    << r << "," << g << "," << b << "," << d << ","
                    << std::endl;
    }
    outputFile.close();

    return 0;
}

CG_Point_3 TriangleCentroid(CG_Triangle tri)
{
    double x = (tri.vertx1.x + tri.vertx2.x + tri.vertx3.x) / 3;
    double y = (tri.vertx1.y + tri.vertx2.y + tri.vertx3.y) / 3;
    double z = (tri.vertx1.z + tri.vertx2.z + tri.vertx3.z) / 3;

    CG_Point_3 centroid(x, y, z);

    return centroid;
}

CG_Vector_3 Normal(CG_Point_3 V1, CG_Point_3 V2, CG_Point_3 V3)
{
    //AB=(a1,a2,a3) AC=(b1,b2,b3) 法向量为AB×AC=(a2b3-a3b2,a3b1-a1b3,a1b2-a2b1)

    double a1 = (V2.x - V1.x);
    double a2 = (V2.y - V1.y);
    double a3 = (V2.z - V1.z);

    double b1 = (V3.x - V1.x);
    double b2 = (V3.y - V1.y);
    double b3 = (V3.z - V1.z);

    double A = (a2 * b3) - (a3 * b2);
    double B = (a3 * b1) - (a1 * b3);
    double C = (a1 * b2) - (a2 * b1);
    CG_Vector_3 N(A, B, C);

    return N;
}

void TriangleMesh(std::vector<CG_Triangle> &triangles, vtkActor *actor)
{
    // 构建mesh
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType index = 0;

    // 创建颜色数组
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("colors");

    for (size_t i = 0; i < triangles.size(); i++)
    {
        // 顶点数据
        double v1x = triangles[i].vertx1.x;
        double v1y = triangles[i].vertx1.y;
        double v1z = triangles[i].vertx1.z;

        double v2x = triangles[i].vertx2.x;
        double v2y = triangles[i].vertx2.y;
        double v2z = triangles[i].vertx2.z;

        double v3x = triangles[i].vertx3.x;
        double v3y = triangles[i].vertx3.y;
        double v3z = triangles[i].vertx3.z;

        // 创建点数据
        points->InsertNextPoint(v1x, v1y, v1z);
        points->InsertNextPoint(v2x, v2y, v2z);
        points->InsertNextPoint(v3x, v3y, v3z);


        // 创建多边形数据
        vtkIdType polygon[3] = { (long long)i * 3 + 0, (long long)i * 3 + 1, (long long)i * 3 + 2 };
        polygons->InsertNextCell(3, polygon);

        // 创建颜色数据
        unsigned char r = FloatToChar(triangles[i].r);
        unsigned char g = FloatToChar(triangles[i].g);
        unsigned char b = FloatToChar(triangles[i].b);
        unsigned char triangleColor[3]{ r, g, b };
        colors->InsertNextTypedTuple(triangleColor);
        colors->InsertNextTypedTuple(triangleColor);
        colors->InsertNextTypedTuple(triangleColor);
    }

    // 创建PolyData对象并设置点和多边形数据
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetPolys(polygons);
    polyData->GetPointData()->SetScalars(colors);

    // 创建Mapper和Actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    actor->SetMapper(mapper);
}

void TrianglePoints(std::vector<CG_Triangle> &triangles, vtkActor *actor)
{
    // 构建points
    long long number = (long long)triangles.size();
    vtkSmartPointer<vtkPolyVertex> polyVertex = vtkSmartPointer<vtkPolyVertex>::New();
    polyVertex->GetPointIds()->SetNumberOfIds(number);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    long long n = 0;
    double x, y, z;
    for (size_t i = 0; i < number; ++i)
    {
        n = (long long)i;
        x = triangles[i].centroid.x;
        y = triangles[i].centroid.y;
        z = triangles[i].centroid.z;

        points->InsertPoint(n, x, y, z);
        polyVertex->GetPointIds()->SetId(n, n);
    }

    vtkSmartPointer<vtkUnstructuredGrid> unstrGrid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    unstrGrid->Allocate(1, 1);
    unstrGrid->SetPoints(points);
    unstrGrid->InsertNextCell(polyVertex->GetCellType(), polyVertex->GetPointIds());

    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
    mapper->SetInputData(unstrGrid);

    actor->SetMapper(mapper);
}

}
