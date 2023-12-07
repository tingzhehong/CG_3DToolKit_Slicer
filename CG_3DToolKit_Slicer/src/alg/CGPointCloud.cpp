#include "CGPointCloud.h"
#include "CGCSV.h"


namespace CG
{

void PointCloud2Color(PointCloudT::Ptr cloud, PointCloudT::Ptr color_cloud)
{
    float max_PtZ = 0;
    float min_PtZ = 0;
    PointCloudMinMaxZ(cloud, max_PtZ, min_PtZ);

    int   indexcolor = 0;
    float ramp = (max_PtZ - min_PtZ) / 50;

    color_cloud->resize(cloud->points.size());
    color_cloud->width = cloud->width;
    color_cloud->height = cloud->height;
    color_cloud->is_dense = false;

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        if (cloud->points[i].z >= min_PtZ)
        {
            indexcolor = ceil((cloud->points[i].z - min_PtZ) / ramp);
            if (indexcolor < 0)  { indexcolor = 0; }
            if (indexcolor > 50) { indexcolor = 50; }

            color_cloud->points[i].x = cloud->points[i].x;
            color_cloud->points[i].y = cloud->points[i].y;
            color_cloud->points[i].z = cloud->points[i].z;
            color_cloud->points[i].r = colorLUT[50 - indexcolor].R * 255;
            color_cloud->points[i].g = colorLUT[50 - indexcolor].G * 255;
            color_cloud->points[i].b = colorLUT[50 - indexcolor].B * 255;
        }
        else
        {
            color_cloud->points[i].x = cloud->points[i].x;
            color_cloud->points[i].y = cloud->points[i].y;
            color_cloud->points[i].z = cloud->points[i].z;
            color_cloud->points[i].r = cloud->points[i].r;
            color_cloud->points[i].g = cloud->points[i].g;
            color_cloud->points[i].b = cloud->points[i].b;
        }
    }
}

void PointCloud2Gray(PointCloudT::Ptr cloud, PointCloudT::Ptr gray_cloud)
{
    float max_PtZ = 0;
    float min_PtZ = 0;
    PointCloudMinMaxZ(cloud, max_PtZ, min_PtZ);

    int   indexcolor = 0;
    float ramp = (max_PtZ - min_PtZ) / 255;

    gray_cloud->resize(cloud->points.size());
    gray_cloud->width = cloud->width;
    gray_cloud->height = cloud->height;
    gray_cloud->is_dense = false;

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        if (cloud->points[i].z >= min_PtZ)
        {
            indexcolor = ceil((cloud->points[i].z - min_PtZ) / ramp);
            if (indexcolor < 0)  { indexcolor = 0; }
            if (indexcolor > 255) { indexcolor = 255; }

            gray_cloud->points[i].x = cloud->points[i].x;
            gray_cloud->points[i].y = cloud->points[i].y;
            gray_cloud->points[i].z = cloud->points[i].z;
            gray_cloud->points[i].r = indexcolor;
            gray_cloud->points[i].g = indexcolor;
            gray_cloud->points[i].b = indexcolor;
        }
        else
        {
            gray_cloud->points[i].x = cloud->points[i].x;
            gray_cloud->points[i].y = cloud->points[i].y;
            gray_cloud->points[i].z = cloud->points[i].z;
            gray_cloud->points[i].r = cloud->points[i].r;
            gray_cloud->points[i].g = cloud->points[i].g;
            gray_cloud->points[i].b = cloud->points[i].b;
        }
    }
}

void PointCloud4Elevation(PointCloudT::Ptr cloud, Mat &ElevationImage)
{
    pcl::PointXYZRGB min_p;
    pcl::PointXYZRGB max_p;
    pcl::getMinMax3D(*cloud, min_p, max_p);

    cv::Mat TempImage;
    cv::flip(ElevationImage, TempImage, 0);

    float XPitch = (max_p.x - min_p.x) / ElevationImage.cols;
    float YPitch = (max_p.y - min_p.y) / ElevationImage.rows;

    if (XPitch <= 0) return;
    if (YPitch <= 0) return;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        int col = static_cast<int>((cloud->points[i].x - min_p.x) / XPitch);
        int row = static_cast<int>((cloud->points[i].y - min_p.y) / YPitch);

        if (col < 0) { col = 0; }
        if (row < 0) { row = 0; }
        if (col > TempImage.cols - 1) { col = TempImage.cols - 1; }
        if (row > TempImage.rows - 1) { row = TempImage.rows - 1; }

        unsigned char b = TempImage.at<cv::Vec3b>(row, col)[0];
        unsigned char g = TempImage.at<cv::Vec3b>(row, col)[1];
        unsigned char r = TempImage.at<cv::Vec3b>(row, col)[2];

        cloud->points[i].r = static_cast<float>(r);
        cloud->points[i].g = static_cast<float>(g);
        cloud->points[i].b = static_cast<float>(b);
    }
}

void PointCloud4Gray(PointCloudT::Ptr cloud, Mat &GrayImage)
{
    pcl::PointXYZRGB min_p;
    pcl::PointXYZRGB max_p;
    pcl::getMinMax3D(*cloud, min_p, max_p);

    cv::Mat TempImage;
    cv::flip(GrayImage, TempImage, 0);

    float XPitch = (max_p.x - min_p.x) / GrayImage.cols;
    float YPitch = (max_p.y - min_p.y) / GrayImage.rows;

    if (XPitch <= 0) return;
    if (YPitch <= 0) return;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        int col = static_cast<int>((cloud->points[i].x - min_p.x) / XPitch);
        int row = static_cast<int>((cloud->points[i].y - min_p.y) / YPitch);

        if (col < 0) { col = 0; }
        if (row < 0) { row = 0; }
        if (col > TempImage.cols - 1) { col = TempImage.cols - 1; }
        if (row > TempImage.rows - 1) { row = TempImage.rows - 1; }

        unsigned char gray = TempImage.at<uchar>(row, col);

        cloud->points[i].r = static_cast<float>(gray);
        cloud->points[i].g = static_cast<float>(gray);
        cloud->points[i].b = static_cast<float>(gray);
    }
}

void PointCloud4Intensity(PointCloudT::Ptr cloud, Mat &IntensityImage)
{
    pcl::PointXYZRGB min_p;
    pcl::PointXYZRGB max_p;
    pcl::getMinMax3D(*cloud, min_p, max_p);

    cv::Mat TempImage;
    cv::flip(IntensityImage, TempImage, 0);

    float XPitch = (max_p.x - min_p.x) / IntensityImage.cols;
    float YPitch = (max_p.y - min_p.y) / IntensityImage.rows;

    if (XPitch <= 0) return;
    if (YPitch <= 0) return;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        int col = static_cast<int>((cloud->points[i].x - min_p.x) / XPitch);
        int row = static_cast<int>((cloud->points[i].y - min_p.y) / YPitch);

        if (col < 0) { col = 0; }
        if (row < 0) { row = 0; }
        if (col > TempImage.cols - 1) { col = TempImage.cols - 1; }
        if (row > TempImage.rows - 1) { row = TempImage.rows - 1; }

        unsigned char intensity = TempImage.at<uchar>(row, col);

        cloud->points[i].r = static_cast<float>(intensity);
        cloud->points[i].g = static_cast<float>(intensity);
        cloud->points[i].b = static_cast<float>(intensity);
    }
}

void PointCloudMinMaxZ(PointCloudT::Ptr cloud, float &CloudMaxZ, float &CloudMinZ)
{
    pcl::PointXYZRGB min_p;	//最小值
    pcl::PointXYZRGB max_p;	//最大值
    pcl::getMinMax3D(*cloud, min_p, max_p);
    CloudMaxZ = max_p.z;
    CloudMinZ = min_p.z;
}

void FromDepthImage2PointCloud(Mat &ImageDepth, float XPitch, float YPitch, float DownLimitThres, float UpLimitThres, PointCloudT::Ptr cloud)
{
    if (ImageDepth.empty()) return;

    int icol = ImageDepth.cols;
    int irow = ImageDepth.rows;

    double dmin_z = 0, dmax_z = 0;
    cv::minMaxIdx(ImageDepth, &dmin_z, &dmax_z);
    float min_z = (float)dmin_z;  if (min_z < DownLimitThres) min_z = DownLimitThres;
    float max_z = (float)dmax_z;  if (max_z > UpLimitThres) max_z = UpLimitThres;
    float ramp = (max_z - min_z) / 50;

    cv::Mat imageDepth = ImageDepth;
    cv::Mat imageDepth_flip;
    cv::flip(imageDepth, imageDepth_flip, 0);

    pcl::PointXYZRGB TimPoint;

    for (int i = 0; i < imageDepth_flip.rows; i++)
    {
        for (int j = 0; j < imageDepth_flip.cols; j++)
        {
            float z = imageDepth_flip.at<float>(i, j);
            if (z <= DownLimitThres) z = NAN;
            if (z >= UpLimitThres) z = NAN;

            int indexcolor = ceil((z - min_z) / ramp);
            if (indexcolor < 0) { indexcolor = 0; }
            if (indexcolor > 50) { indexcolor = 50; }

            float r = colorLUT[50 - indexcolor].R * 255;
            float g = colorLUT[50 - indexcolor].G * 255;
            float b = colorLUT[50 - indexcolor].B * 255;

            TimPoint.r = r;
            TimPoint.g = g;
            TimPoint.b = b;

            TimPoint.x = j * XPitch;
            TimPoint.y = i * YPitch;
            TimPoint.z = z;

            cloud->push_back(TimPoint);
        }
    }

    cloud->width = icol;
    cloud->height = irow;
    cloud->is_dense = false;
}

void DownSampleFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr sub_cloud, float leaf)
{
    pcl::PCLPointCloud2::Ptr blob_cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr filtered_cloud(new pcl::PCLPointCloud2);

    pcl::toPCLPointCloud2(*cloud, *blob_cloud);
    pcl::VoxelGrid<pcl::PCLPointCloud2> VoxeGridlFilter;
    VoxeGridlFilter.setInputCloud(blob_cloud);
    VoxeGridlFilter.setLeafSize(leaf, leaf, leaf);
    VoxeGridlFilter.filter(*filtered_cloud);

    pcl::fromPCLPointCloud2(*filtered_cloud, *sub_cloud);
}

void SORFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr filter_cloud, const float thres)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(6);
    sor.setStddevMulThresh(thres);
    sor.filter(*filter_cloud);
}

void PassThroughFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr filter_cloud, const String Direction, const float LimitsDown, const float LimitsUp)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(Direction);
    pass.setFilterLimits(LimitsDown, LimitsUp);
    pass.filter(*filter_cloud);
}

void LoadPCDFile(const string filename, vtkActor *actor)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud); if (cloud->empty()) return;
    g_PointCloud = cloud;
    long long number = (long long)cloud->size();

    vtkSmartPointer<vtkLookupTable> lut =vtkSmartPointer< vtkLookupTable >::New();
    lut->SetNumberOfTableValues(number);
    lut->Build();

    vtkSmartPointer<vtkPolyVertex> polyVertex = vtkSmartPointer<vtkPolyVertex>::New();
    polyVertex->GetPointIds()->SetNumberOfIds(number);

    vtkSmartPointer<vtkDoubleArray> pointsScalars = vtkSmartPointer<vtkDoubleArray>::New();
    pointsScalars->SetNumberOfTuples(number);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    long long n = 0;
    double x, y, z;
    double r, g, b;
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        n = (long long)i;
        x = (double)cloud->points[i].x;
        y = (double)cloud->points[i].y;
        z = (double)cloud->points[i].z;
        r = (double)cloud->points[i].r;
        g = (double)cloud->points[i].g;
        b = (double)cloud->points[i].b;

        points->InsertPoint(n, x, y, z);
        lut->SetTableValue(n, r / 255, g / 255, b / 255, 1);
        polyVertex->GetPointIds()->SetId(n, n); //第1个参数是几何point的Id号，第2个参数是拓扑中的Id号
        pointsScalars->InsertValue(n, n);       //第1个参数是points点的Id，第2个参数是该点的属性值
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    g_PolyData = polyData;

    vtkSmartPointer<vtkUnstructuredGrid> unstrGrid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    unstrGrid->Allocate(1, 1);
    unstrGrid->SetPoints(points);
    unstrGrid->GetPointData()->SetScalars(pointsScalars);
    unstrGrid->InsertNextCell(polyVertex->GetCellType(), polyVertex->GetPointIds()); //设置映射器

    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
    mapper->SetInputData(unstrGrid);
    mapper->ScalarVisibilityOn();
    mapper->SetScalarRange(0, number - 1);
    mapper->SetLookupTable(lut);

    actor->SetMapper(mapper);
    actor->GetProperty()->SetRepresentationToPoints();
    actor->GetProperty()->SetPointSize(1);
}

void LoadCSVFile(const string filename, vtkActor *actor)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    float X = 0;
    float Y = 0;
    float XPitch = 0;
    float YPitch = 0;
    int   rowNum = 0;
    int   colNum = 0;
    GetCSVSize(filename, X, Y, XPitch, YPitch, rowNum, colNum);
    CSVFile2PointCloud(filename, X, Y, XPitch, YPitch, rowNum, colNum, cloud);
    g_PointCloud = cloud;
    long long number = (long long)cloud->size();

    vtkSmartPointer<vtkLookupTable> lut =vtkSmartPointer< vtkLookupTable >::New();
    lut->SetNumberOfTableValues(number);
    lut->Build();

    vtkSmartPointer<vtkPolyVertex> polyVertex = vtkSmartPointer<vtkPolyVertex>::New();
    polyVertex->GetPointIds()->SetNumberOfIds(number);

    vtkSmartPointer<vtkDoubleArray> pointsScalars = vtkSmartPointer<vtkDoubleArray>::New();
    pointsScalars->SetNumberOfTuples(number);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    long long n = 0;
    double x, y, z;
    double r, g, b;
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        n = (long long)i;
        x = (double)cloud->points[i].x;
        y = (double)cloud->points[i].y;
        z = (double)cloud->points[i].z;
        r = (double)cloud->points[i].r;
        g = (double)cloud->points[i].g;
        b = (double)cloud->points[i].b;

        points->InsertPoint(n, x, y, z);
        lut->SetTableValue(n, r / 255, g / 255, b / 255, 1);
        polyVertex->GetPointIds()->SetId(n, n); //第1个参数是几何point的Id号，第2个参数是拓扑中的Id号
        pointsScalars->InsertValue(n, n);       //第1个参数是points点的Id，第2个参数是该点的属性值
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    g_PolyData = polyData;

    vtkSmartPointer<vtkUnstructuredGrid> unstrGrid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    unstrGrid->Allocate(1, 1);
    unstrGrid->SetPoints(points);
    unstrGrid->GetPointData()->SetScalars(pointsScalars);
    unstrGrid->InsertNextCell(polyVertex->GetCellType(), polyVertex->GetPointIds()); //设置映射器

    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
    mapper->SetInputData(unstrGrid);
    mapper->ScalarVisibilityOn();
    mapper->SetScalarRange(0, number - 1);
    mapper->SetLookupTable(lut);

    actor->SetMapper(mapper);
    actor->GetProperty()->SetRepresentationToPoints();
    actor->GetProperty()->SetPointSize(1);
}

void LoadTXTFile(const string filename, vtkActor *actor)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB TimPoint;

    std::ifstream filestream(filename.c_str());
    std::string line;
    while (std::getline(filestream, line))
    {
        double x, y, z;
        std::stringstream linestream;
        linestream << line;
        linestream >> x >> y >> z;

        TimPoint.x = x;
        TimPoint.y = y;
        TimPoint.z = z;
        TimPoint.r = 255;
        TimPoint.g = 255;
        TimPoint.b = 255;

        cloud->push_back(TimPoint);
    }
    filestream.close();

    g_PointCloud = cloud;
    long long number = (long long)cloud->size();

    vtkSmartPointer<vtkLookupTable> lut =vtkSmartPointer< vtkLookupTable >::New();
    lut->SetNumberOfTableValues(number);
    lut->Build();

    vtkSmartPointer<vtkPolyVertex> polyVertex = vtkSmartPointer<vtkPolyVertex>::New();
    polyVertex->GetPointIds()->SetNumberOfIds(number);

    vtkSmartPointer<vtkDoubleArray> pointsScalars = vtkSmartPointer<vtkDoubleArray>::New();
    pointsScalars->SetNumberOfTuples(number);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    long long n = 0;
    double x, y, z;
    double r, g, b;
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        n = (long long)i;
        x = (double)cloud->points[i].x;
        y = (double)cloud->points[i].y;
        z = (double)cloud->points[i].z;
        r = (double)cloud->points[i].r;
        g = (double)cloud->points[i].g;
        b = (double)cloud->points[i].b;

        points->InsertPoint(n, x, y, z);
        lut->SetTableValue(n, r / 255, g / 255, b / 255, 1);
        polyVertex->GetPointIds()->SetId(n, n); //第1个参数是几何point的Id号，第2个参数是拓扑中的Id号
        pointsScalars->InsertValue(n, n);       //第1个参数是points点的Id，第2个参数是该点的属性值
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    g_PolyData = polyData;

    vtkSmartPointer<vtkUnstructuredGrid> unstrGrid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    unstrGrid->Allocate(1, 1);
    unstrGrid->SetPoints(points);
    unstrGrid->GetPointData()->SetScalars(pointsScalars);
    unstrGrid->InsertNextCell(polyVertex->GetCellType(), polyVertex->GetPointIds()); //设置映射器

    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
    mapper->SetInputData(unstrGrid);
    mapper->ScalarVisibilityOn();
    mapper->SetScalarRange(0, number - 1);
    mapper->SetLookupTable(lut);

    actor->SetMapper(mapper);
    actor->GetProperty()->SetRepresentationToPoints();
    actor->GetProperty()->SetPointSize(1);
}

void VTKPointCloudElevation(PointCloudT::Ptr cloud, vtkActor *actor)
{
    if (cloud->empty()) return;
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

    long long n = 0;
    double x, y, z;
    double r, g, b;
    double MinZ = min_pt.z, MaxZ = max_pt.z;
    for (size_t i = 0; i < cloud->size(); ++i)
    {

        n = (double)i;
        x = (double)cloud->points[i].x;
        y = (double)cloud->points[i].y;
        z = (double)cloud->points[i].z;
        r = (double)cloud->points[i].r;
        g = (double)cloud->points[i].g;
        b = (double)cloud->points[i].b;

        points->InsertPoint(n, x, y, z);
     }

     vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
     polyData->SetPoints(points);
     g_PolyData = polyData;

     vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
     glyphFilter->SetInputData(polyData);
     glyphFilter->Update();

     vtkSmartPointer<vtkElevationFilter> coloredGrid = vtkElevationFilter::New();
     coloredGrid->SetInputConnection(glyphFilter->GetOutputPort());
     coloredGrid->SetLowPoint(0, 0, MaxZ);
     coloredGrid->SetHighPoint(0, 0, MinZ);

     vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
     mapper->SetInputConnection(coloredGrid->GetOutputPort());

     actor->SetMapper(mapper);
}

void VTKPointCloudGray(PointCloudT::Ptr cloud, vtkActor *actor)
{
    if (cloud->empty()) return;
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    int   indexcolor = 0;
    float ramp = (max_pt.z - min_pt.z) / 255;

    long long number = (long long)cloud->size();

    vtkSmartPointer<vtkLookupTable> lut =vtkSmartPointer< vtkLookupTable >::New();
    lut->SetNumberOfTableValues(number);
    lut->Build();

    vtkSmartPointer<vtkPolyVertex> polyVertex = vtkSmartPointer<vtkPolyVertex>::New();
    polyVertex->GetPointIds()->SetNumberOfIds(number);

    vtkSmartPointer<vtkDoubleArray> pointsScalars = vtkSmartPointer<vtkDoubleArray>::New();
    pointsScalars->SetNumberOfTuples(number);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    long long n = 0;
    double x, y, z;
    double r, g, b;
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        indexcolor = ceil((cloud->points[i].z - min_pt.z) / ramp);
        if (indexcolor < 0)  { indexcolor = 0; }
        if (indexcolor > 255) { indexcolor = 255; }

        n = (long long)i;
        x = (double)cloud->points[i].x;
        y = (double)cloud->points[i].y;
        z = (double)cloud->points[i].z;
        r = (double)indexcolor;
        g = (double)indexcolor;
        b = (double)indexcolor;

        points->InsertPoint(n, x, y, z);
        lut->SetTableValue(n, r / 255, g / 255, b / 255, 1);
        polyVertex->GetPointIds()->SetId(n, n); //第1个参数是几何point的Id号，第2个参数是拓扑中的Id号
        pointsScalars->InsertValue(n, n);       //第1个参数是points点的Id，第2个参数是该点的属性值
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    g_PolyData = polyData;

    vtkSmartPointer<vtkUnstructuredGrid> unstrGrid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    unstrGrid->Allocate(1, 1);
    unstrGrid->SetPoints(points);
    unstrGrid->GetPointData()->SetScalars(pointsScalars);
    unstrGrid->InsertNextCell(polyVertex->GetCellType(), polyVertex->GetPointIds()); //设置映射器

    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
    mapper->SetInputData(unstrGrid);
    mapper->ScalarVisibilityOn();
    mapper->SetScalarRange(0, number - 1);
    mapper->SetLookupTable(lut);

    actor->SetMapper(mapper);
    actor->GetProperty()->SetRepresentationToPoints();
    actor->GetProperty()->SetPointSize(1);
}

void VTKPointCloudIntensity(PointCloudT::Ptr cloud, vtkActor *actor)
{
    if (cloud->empty()) return;
    long long number = (long long)cloud->size();

    vtkSmartPointer<vtkLookupTable> lut =vtkSmartPointer< vtkLookupTable >::New();
    lut->SetNumberOfTableValues(number);
    lut->Build();

    vtkSmartPointer<vtkPolyVertex> polyVertex = vtkSmartPointer<vtkPolyVertex>::New();
    polyVertex->GetPointIds()->SetNumberOfIds(number);

    vtkSmartPointer<vtkDoubleArray> pointsScalars = vtkSmartPointer<vtkDoubleArray>::New();
    pointsScalars->SetNumberOfTuples(number);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    long long n = 0;
    double x, y, z;
    double r, g, b;
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        n = (long long)i;
        x = (double)cloud->points[i].x;
        y = (double)cloud->points[i].y;
        z = (double)cloud->points[i].z;
        r = (double)cloud->points[i].r;
        g = (double)cloud->points[i].g;
        b = (double)cloud->points[i].b;

        points->InsertPoint(n, x, y, z);
        lut->SetTableValue(n, r / 255, g / 255, b / 255, 1);
        polyVertex->GetPointIds()->SetId(n, n); //第1个参数是几何point的Id号，第2个参数是拓扑中的Id号
        pointsScalars->InsertValue(n, n);       //第1个参数是points点的Id，第2个参数是该点的属性值
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    g_PolyData = polyData;

    vtkSmartPointer<vtkUnstructuredGrid> unstrGrid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    unstrGrid->Allocate(1, 1);
    unstrGrid->SetPoints(points);
    unstrGrid->GetPointData()->SetScalars(pointsScalars);
    unstrGrid->InsertNextCell(polyVertex->GetCellType(), polyVertex->GetPointIds()); //设置映射器

    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
    mapper->SetInputData(unstrGrid);
    mapper->ScalarVisibilityOn();
    mapper->SetScalarRange(0, number - 1);
    mapper->SetLookupTable(lut);

    actor->SetMapper(mapper);
    actor->GetProperty()->SetRepresentationToPoints();
    actor->GetProperty()->SetPointSize(1);
}

bool IsOrderPointCloud()
{
    if (g_PointCloud->height > 1 && g_PointCloud->width > 1) {
        return true;
    } else {
        return false;
    }
    return false;
}

void LoadPLYFile(const string filename, vtkActor *actor)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile<pcl::PointXYZRGB>(filename, *cloud); if (cloud->empty()) return;

    g_PointCloud = cloud;
    long long number = (long long)cloud->size();

    vtkSmartPointer<vtkLookupTable> lut =vtkSmartPointer< vtkLookupTable >::New();
    lut->SetNumberOfTableValues(number);
    lut->Build();

    vtkSmartPointer<vtkPolyVertex> polyVertex = vtkSmartPointer<vtkPolyVertex>::New();
    polyVertex->GetPointIds()->SetNumberOfIds(number);

    vtkSmartPointer<vtkDoubleArray> pointsScalars = vtkSmartPointer<vtkDoubleArray>::New();
    pointsScalars->SetNumberOfTuples(number);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    long long n = 0;
    double x, y, z;
    double r, g, b;
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        n = (long long)i;
        x = (double)cloud->points[i].x;
        y = (double)cloud->points[i].y;
        z = (double)cloud->points[i].z;
        r = (double)cloud->points[i].r;
        g = (double)cloud->points[i].g;
        b = (double)cloud->points[i].b;

        points->InsertPoint(n, x, y, z);
        lut->SetTableValue(n, r / 255, g / 255, b / 255, 1);
        polyVertex->GetPointIds()->SetId(n, n); //第1个参数是几何point的Id号，第2个参数是拓扑中的Id号
        pointsScalars->InsertValue(n, n);       //第1个参数是points点的Id，第2个参数是该点的属性值
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    g_PolyData = polyData;

    vtkSmartPointer<vtkUnstructuredGrid> unstrGrid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    unstrGrid->Allocate(1, 1);
    unstrGrid->SetPoints(points);
    unstrGrid->GetPointData()->SetScalars(pointsScalars);
    unstrGrid->InsertNextCell(polyVertex->GetCellType(), polyVertex->GetPointIds()); //设置映射器

    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
    mapper->SetInputData(unstrGrid);
    mapper->ScalarVisibilityOn();
    mapper->SetScalarRange(0, number - 1);
    mapper->SetLookupTable(lut);

    actor->SetMapper(mapper);
    actor->GetProperty()->SetRepresentationToPoints();
    actor->GetProperty()->SetPointSize(1);
}

void LoadSTLFile(const string filename, vtkActor *actor)
{
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    actor->SetMapper(mapper);
    actor->GetProperty()->SetRepresentationToSurface();

    vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
    vtkSmartPointer<vtkPoints> points = polydata->GetPoints();
    long long number = points->GetNumberOfPoints();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB TimPoint;

    for (long long i = 1; i < number; ++i)
    {
        double* point = points->GetPoint(i);

        TimPoint.x = (float)point[0];
        TimPoint.y = (float)point[1];
        TimPoint.z = (float)point[2];
        TimPoint.r = 255;
        TimPoint.g = 255;
        TimPoint.b = 255;

        cloud->push_back(TimPoint);
    }
    g_PointCloud = cloud;
    g_PolyData = polydata;
}

void LoadOBJFile(const string filename, vtkActor *actor)
{
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    actor->SetMapper(mapper);
    actor->GetProperty()->SetRepresentationToSurface();

    vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
    vtkSmartPointer<vtkPoints> points = polydata->GetPoints();
    long number = points->GetNumberOfPoints();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB TimPoint;

    for (long long i = 1; i < number; ++i)
    {
        double* point = points->GetPoint(i);

        TimPoint.x = (float)point[0];
        TimPoint.y = (float)point[1];
        TimPoint.z = (float)point[2];
        TimPoint.r = 255;
        TimPoint.g = 255;
        TimPoint.b = 255;

        cloud->push_back(TimPoint);
    }
    g_PointCloud = cloud;
    g_PolyData = polydata;
}

}
