#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::CreateImageALL(PointCloudT::Ptr cloud, cv::Mat &imgDepth, cv::Mat &imgGray, cv::Mat &imgIntensity, const float &XPitch, const float &YPitch, const int &rowNum, const int &colNum)
{
	pcl::PointXYZRGB min;	//最小值
	pcl::PointXYZRGB max;	//最大值
	pcl::getMinMax3D(*cloud, min, max);

    int   row = rowNum;
    int   col = colNum ;
    float XPITCH = (max.x - min.x) / colNum;
    float YPITCH = (max.y - min.y) / rowNum;
	std::cout << "Create Image..." << endl;
	std::cout << colNum << "  " << rowNum << endl;

	float factorZ;
	factorZ = (255 - 0) / (max.z - min.z);	//灰度范围
	
	cv::Mat imageDepth(row, col, CV_32FC1, Scalar(min.z));		//填充图像
	cv::Mat imageDepth_flip(row, col, CV_32FC1, Scalar(min.z));	//填充图像

	cv::Mat imageGray(row, col, CV_8UC1, Scalar(0));		//填充图像
	cv::Mat imageGray_flip(row, col, CV_8UC1, Scalar(0));	//填充图像
	
	cv::Mat imageIntensity(row, col, CV_8UC1, Scalar(0));		//填充图像
	cv::Mat imageIntensity_flip(row, col, CV_8UC1, Scalar(0));	//填充图像

    for (size_t i = 0; i < cloud->size(); i++)
	{
        int icol = int((cloud->points[i].x - min.x) / XPITCH);
		int irow = int((cloud->points[i].y - min.y) / YPITCH);

        if (icol < col && irow < row && icol > 0 && irow > 0)
		{
            imageDepth.at<float>(irow, icol) = cloud->points[i].z;

			int igray = int((cloud->points[i].z - min.z) * factorZ);
			imageGray.at<uchar>(irow, icol) = igray;
			
			int intensity = int(cloud->points[i].r);
			imageIntensity.at<uchar>(irow, icol) = intensity;

            if (icol - 1 < col && irow - 1 < row && icol - 1 > 0 && irow - 1 > 0)
            {
                if (imageDepth.at<float>(irow - 1, icol) == min.z)
                    imageDepth.at<float>(irow - 1, icol) = cloud->points[i].z;
                if (imageDepth.at<float>(irow, icol - 1) == min.z)
                    imageDepth.at<float>(irow, icol - 1) = cloud->points[i].z;
                if (imageDepth.at<float>(irow - 1, icol - 1) == min.z)
                    imageDepth.at<float>(irow - 1, icol - 1) = cloud->points[i].z;

                if (imageGray.at<uchar>(irow - 1, icol) == 0)
                    imageGray.at<uchar>(irow - 1, icol) = igray;
                if (imageGray.at<uchar>(irow, icol - 1) == 0)
                    imageGray.at<uchar>(irow, icol - 1) = igray;
                if (imageGray.at<uchar>(irow - 1, icol - 1) == 0)
                    imageGray.at<uchar>(irow - 1, icol - 1) = igray;

                if (imageIntensity.at<uchar>(irow - 1, icol) == 0)
                    imageIntensity.at<uchar>(irow - 1, icol) = intensity;
                if (imageIntensity.at<uchar>(irow, icol - 1) == 0)
                    imageIntensity.at<uchar>(irow, icol - 1) = intensity;
                if (imageIntensity.at<uchar>(irow - 1, icol - 1) == 0)
                    imageIntensity.at<uchar>(irow - 1, icol - 1) = intensity;
            }

            if (icol + 1 < col && irow + 1 < row && icol + 1 > 0 && irow + 1 > 0)
            {
                if (imageDepth.at<float>(irow + 1, icol) == min.z)
                    imageDepth.at<float>(irow + 1, icol) = cloud->points[i].z;
                if (imageDepth.at<float>(irow, icol + 1) == min.z)
                    imageDepth.at<float>(irow, icol + 1) = cloud->points[i].z;
                if (imageDepth.at<float>(irow + 1, icol + 1) == min.z)
                    imageDepth.at<float>(irow + 1, icol + 1) = cloud->points[i].z;

                if (imageGray.at<uchar>(irow + 1, icol) == 0)
                    imageGray.at<uchar>(irow + 1, icol) = igray;
                if (imageGray.at<uchar>(irow, icol + 1) == 0)
                    imageGray.at<uchar>(irow, icol + 1) = igray;
                if (imageGray.at<uchar>(irow + 1, icol + 1) == 0)
                    imageGray.at<uchar>(irow + 1, icol + 1) = igray;

                if (imageIntensity.at<uchar>(irow + 1, icol) == 0)
                    imageIntensity.at<uchar>(irow + 1, icol) = intensity;
                if (imageIntensity.at<uchar>(irow, icol + 1) == 0)
                    imageIntensity.at<uchar>(irow, icol + 1) = intensity;
                if (imageIntensity.at<uchar>(irow + 1, icol + 1) == 0)
                    imageIntensity.at<uchar>(irow + 1, icol + 1) = intensity;
            }

            if (icol - 1 < col && irow + 1 < row && icol - 1 > 0 && irow + 1 > 0)
            {
                if (imageDepth.at<float>(irow + 1, icol - 1) == min.z)
                    imageDepth.at<float>(irow + 1, icol - 1) = cloud->points[i].z;

                if (imageGray.at<uchar>(irow + 1, icol - 1) == 0)
                    imageGray.at<uchar>(irow + 1, icol - 1) = igray;

                if (imageIntensity.at<uchar>(irow + 1, icol - 1) == 0)
                    imageIntensity.at<uchar>(irow + 1, icol - 1) = intensity;
            }

            if (icol + 1 < col && irow - 1 < row && icol + 1 > 0 && irow - 1 > 0)
            {
                if (imageDepth.at<float>(irow - 1, icol + 1) == min.z)
                    imageDepth.at<float>(irow - 1, icol + 1) = cloud->points[i].z;

                if (imageGray.at<uchar>(irow - 1, icol + 1) == 0)
                    imageGray.at<uchar>(irow - 1, icol + 1) = igray;

                if (imageIntensity.at<uchar>(irow - 1, icol + 1) == 0)
                    imageIntensity.at<uchar>(irow - 1, icol + 1) = intensity;
            }
        }
    }
    cv::Size size_flip = XPITCH >= YPITCH ? cv::Size(col * (XPITCH / YPITCH), row) : cv::Size(col, row * (YPITCH / XPITCH));

    cv::flip(imageDepth, imageDepth_flip, 0);	//上下翻转图像
	cv::resize(imageDepth_flip, imgDepth, size_flip);

    cv::flip(imageGray, imageGray_flip, 0);	//上下翻转图像
    cv::resize(imageGray_flip, imgGray, size_flip);
	
	cv::flip(imageIntensity, imageIntensity_flip, 0);	//上下翻转图像
	cv::resize(imageIntensity_flip, imgIntensity, size_flip);
}

void alg::GrayMat2ColorMat(cv::Mat &grayImage, cv::Mat &colorImage)
{
    int indexcolor = 0;

    cv::Mat img(grayImage.rows, grayImage.cols, CV_8UC3, cv::Scalar(0));

    for (int i = 0; i < grayImage.rows; ++i)
    {
        for (int j = 0; j < grayImage.cols; ++j)
        {
            indexcolor = ceil(static_cast<float>(grayImage.at<uchar>(i, j)) / (255.0f / 50));
            if (indexcolor < 0)  { indexcolor = 0; }
            if (indexcolor > 50) { indexcolor = 50; }

            img.at<cv::Vec3b>(i, j)[0] = colorLUT[50 - indexcolor].B * 255.0f;
            img.at<cv::Vec3b>(i, j)[1] = colorLUT[50 - indexcolor].G * 255.0f;
            img.at<cv::Vec3b>(i, j)[2] = colorLUT[50 - indexcolor].R * 255.0f;
        }
    }

    colorImage = img.clone();
}

void alg::DepthMat2ColorMat(cv::Mat &depthImage, cv::Mat &colorImage)
{
    double min_z = 0, max_z = 0;
    cv::minMaxIdx(depthImage, &min_z, &max_z);
    float ramp = (max_z - min_z) / 50;

    int indexcolor = 0;
    cv::Mat img(depthImage.rows, depthImage.cols, CV_8UC3, cv::Scalar(0));

    for (int i = 0; i < depthImage.rows; ++i)
    {
        for (int j = 0; j < depthImage.cols; ++j)
        {
            indexcolor = ceil((depthImage.at<float>(i, j) - min_z) / ramp);
            if (indexcolor < 0) { indexcolor = 0; }
            if (indexcolor > 50) { indexcolor = 50; }

            img.at<cv::Vec3b>(i, j)[0] = colorLUT[50 - indexcolor].B * 255.0f;
            img.at<cv::Vec3b>(i, j)[1] = colorLUT[50 - indexcolor].G * 255.0f;
            img.at<cv::Vec3b>(i, j)[2] = colorLUT[50 - indexcolor].R * 255.0f;
        }
    }

    colorImage = img.clone();
}
