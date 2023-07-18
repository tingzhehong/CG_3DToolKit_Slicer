#ifndef ALGORITHMFUCTION_H
#define ALGORITHMFUCTION_H

#include <AlgorithmInterface.h>
#include <pcl/filters/voxel_grid.h>

namespace alg
{
    /**
    * @brief        体素滤波 VoxelFilter
    * @param[in]
    * @param[out]
    * @return
    * @author
    **/
    void VoxelFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr sub_cloud, const float leaf);
}

#endif // ALGORITHMFUCTION_H
