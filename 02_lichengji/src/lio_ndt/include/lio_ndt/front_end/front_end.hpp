#pragma once

#include <deque>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <lio_ndt/sensor_data/cloud_data.hpp>

#include "lio_ndt/method/optimized_ICP_GN.h"
#include "lio_ndt/method/ndt_registration.h"
#include "lio_ndt/method/common.h"

namespace lio_ndt
{
    class FrontEnd
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        class Frame
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            CloudData cloud_data;
        };

        enum RegistrationMethod
        {
            ICP_GN = 0,
            NDT = 1
        };

    public:
        FrontEnd();
        
        Eigen::Matrix4f Update(const CloudData& cloud_data);
        bool SetInitPose(const Eigen::Matrix4f& init_pose);
        bool SetPredictPose(const Eigen::Matrix4f& predict_pose);
        
        // 设置配准方法
        void SetRegistrationMethod(RegistrationMethod method);
        
        // 全局地图重建相关
        void SaveGlobalMap(const std::string& file_path);
        void ResetGlobalMap();
        size_t GetGlobalMapSize() const;
        
        // NDT配准结果保存相关
        void SaveNDTRegistrationResult(const std::string& file_path);
        bool GetNDTRegistrationResult(CloudData::CLOUD_PTR& ndt_result_ptr);

        bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
        bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
        bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);

    private:
        void UpdateNewFrame(const Frame& new_key_frame);
        void UpdateGlobalMap(const Frame& new_key_frame);

    private:
        // 变量
        pcl::VoxelGrid<CloudData::POINT> cloud_filter_;
        pcl::VoxelGrid<CloudData::POINT> local_map_filter_;
        pcl::VoxelGrid<CloudData::POINT> display_filter_;
        pcl::VoxelGrid<CloudData::POINT> global_map_filter_;
        
        // 配准算法
        OptimizedICPGN icp_opti_;
        NDTRegistration ndt_registration_;
        RegistrationMethod registration_method_;
        
        std::deque<Frame, Eigen::aligned_allocator<Frame>> local_map_frames_;
        std::deque<Frame, Eigen::aligned_allocator<Frame>> global_map_frames_;

        bool has_new_local_map_ = false;
        bool has_new_global_map_ = false;

        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR result_cloud_ptr_;
        Frame current_frame_;

        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();
    };
}