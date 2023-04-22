//CGCSV.cpp
//

#include "CGCSV.h"

namespace CG
{

void GetCSVSize(const string &fileCSV, float &X, float &Y, float &XPitch, float &YPitch, int &rowNum, int &colNum)
{
	ifstream inputFile(fileCSV);
	string oneLine;
	int lineCount = 0;
	while (getline(inputFile, oneLine))
	{
		++lineCount;
		if (lineCount < 4)
			continue;
		if (lineCount > 5)
			break;
		istringstream iOneLine(oneLine);

		if (lineCount == 5)
		{
			int count = 0;
			string str;
			while (getline(iOneLine, str, ','))
			{
				++count;

				if (count < 1)
				{
					continue;
				}
				else if (count == 1)
				{
					stringstream ss;
					ss << str;
					ss >> X;
				}
				else if (count == 2)
				{
					stringstream ss;
					ss << str;
					ss >> Y;;
				}
				else if (count == 3)
				{
					stringstream ss;
					ss << str;
					ss >> XPitch;
				}
				else if (count == 4)
				{
					stringstream ss;
					ss << str;
					ss >> YPitch;
				}
				else if (count == 5)
				{
					stringstream ss;
					ss << str;
					ss >> rowNum;
				}
				else if (count == 6)
				{
					stringstream ss;
					ss << str;
					ss >> colNum;
				}
				else
					break;
			}
		}
	}
	inputFile.close();
}

void CSVFile2PointCloud(const string &fileCSV, float &X, float &Y, float &XPitch, float &YPitch, int &rowNum, int &colNum, PointCloudT::Ptr cloud)
{
	vector<float> VecX, VecY, VecZ, VecI;

	std::cout << "Read CSV File:  " << fileCSV << endl;

	ifstream inputFile(fileCSV);
	string oneLine;
	int lineCount = 0;
	int rowCount = 0;
	float fData;

	while (getline(inputFile, oneLine))
	{
		++lineCount;
		if (lineCount <= 5)
			continue;

		int colCount = 0;

		istringstream iOneLine(oneLine);
		string str;

		if (rowCount < rowNum)
		{
			while (getline(iOneLine, str, ','))
			{
				float f;
				if (str == "")
				{
					f = 0.f;
					fData = f;
				}
				else
				{
					f = atof(str.c_str());
					fData = f;
				}
				VecX.push_back(colCount * XPitch + X);
				VecY.push_back(0 - (rowCount * YPitch + Y));
				VecZ.push_back(fData);

				++colCount;
				if (colCount >= colNum)
					break;
			}
			++rowCount;
		}
		else
		{
			while (getline(iOneLine, str, ','))
			{
				float f;
				int i;
				if (str == "")
				{
					f = 0.f;
					i = 0;
				}
				else
				{
					f = atof(str.c_str());
					i = int(f / 4);	//0~1023 To 0~255
									//i = int(f / 1);
				}
				VecI.push_back(i);

				++colCount;
				if (colCount >= colNum)
					break;
			}
			//VecI.pop_back();
			++rowCount;
		}

		int Processing = rowCount;

		if (Processing <= rowNum)
		{
			if (Processing == rowNum / 10 * 1)
				cout << "...10%...";
			if (Processing == rowNum / 10 * 2)
				cout << "...20%...";
			if (Processing == rowNum / 10 * 3)
				cout << "...30%...";
			if (Processing == rowNum / 10 * 4)
				cout << "...40%...";
			if (Processing == rowNum / 10 * 5)
				cout << "...50%...";
			if (Processing == rowNum / 10 * 6)
				cout << "...60%...";
			if (Processing == rowNum / 10 * 7)
				cout << "...70%...";
			if (Processing == rowNum / 10 * 8)
				cout << "...80%...";
			if (Processing == rowNum / 10 * 9)
				cout << "...90%...";
			if (Processing == rowNum / 10 * 10)
				cout << "...100%..." << endl;
		}
		else
		{
			if (Processing == rowNum / 10 * 1 + rowNum)
				cout << "...10%...";
			if (Processing == rowNum / 10 * 2 + rowNum)
				cout << "...20%...";
			if (Processing == rowNum / 10 * 3 + rowNum)
				cout << "...30%...";
			if (Processing == rowNum / 10 * 4 + rowNum)
				cout << "...40%...";
			if (Processing == rowNum / 10 * 5 + rowNum)
				cout << "...50%...";
			if (Processing == rowNum / 10 * 6 + rowNum)
				cout << "...60%...";
			if (Processing == rowNum / 10 * 7 + rowNum)
				cout << "...70%...";
			if (Processing == rowNum / 10 * 8 + rowNum)
				cout << "...80%...";
			if (Processing == rowNum / 10 * 9 + rowNum)
				cout << "...90%...";
			if (Processing == rowNum / 10 * 10 + rowNum)
				cout << "...100%..." << endl;
		}

		if (rowCount >= rowNum * 2)
			break;
	}
    inputFile.close();

	cloud->resize(VecZ.size());
    cloud->width = colNum;
    cloud->height = rowNum;
    cloud->is_dense = false;

	for (size_t i = 0; i < VecZ.size(); ++i)
	{
		if (VecZ[i] > -99 && VecZ[i] < 99)
		{
			cloud->points[i].x = VecX[i];
			cloud->points[i].y = VecY[i];
			cloud->points[i].z = VecZ[i];
			cloud->points[i].r = VecI[i];
			cloud->points[i].g = VecI[i];
			cloud->points[i].b = VecI[i];
		}
	}
    g_PointCloud = cloud;
}

void PointCloud2CSVFile(const string &fileCSV, float &X, float &Y, float &XPitch, float &YPitch, int &rowNum, int &colNum, PointCloudT::Ptr cloud)
{
	time_t NowTime = GetCurrentTime();
	tm *pNowTime;
	time(&NowTime);
	pNowTime = localtime(&NowTime);

	std::string ThisTime = std::to_string(pNowTime->tm_year + 1900) + "-" + std::to_string(pNowTime->tm_mon + 1) + "-" + std::to_string(pNowTime->tm_mday) + "  " +
                           std::to_string(pNowTime->tm_hour) + ":" + std::to_string(pNowTime->tm_min) + ":" + std::to_string(pNowTime->tm_sec);

	ofstream outputFile(fileCSV);

	outputFile << "Machine: CGMACHINE" << endl;
	outputFile << "Created By: CGSOFT//USER Tim" << endl;
	outputFile << "Created On: " << ThisTime << endl;
	outputFile << "X,Y,XPitch,YPitch,Row,Column" << endl;
	outputFile << X << "," << Y << "," << XPitch << "," << YPitch << "," << rowNum << "," << colNum << "," << endl;

    for (int i = rowNum - 1; i >= 0; i--)
	{
        for (int j = 0; j < colNum - 1; j++)
		{
			outputFile << cloud->points[i * colNum + j].z << ",";
		}
		outputFile << cloud->points[i * colNum + colNum].z << endl;
	}

    for (int m = rowNum - 1; m >= 0; m--)
	{
        for (int n = 0; n < colNum - 1; n++)
		{
			outputFile << cloud->points[m * colNum + n].r * 4 << ",";
		}
        outputFile << cloud->points[m * colNum + colNum - 1].r * 4 << endl;
	}

	outputFile.close();
}

}
