#include <lio_ndt/front_end/front_end.hpp>
#include <lio_ndt/global_defination/global_defination.h>

#include <cmath>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <glog/logging.h>

namespace lio_ndt
{
    FrontEnd::FrontEnd():icp_opti_(OptimizedICPGN()),
                         ndt_registration_(NDTRegistration()),
                         registration_method_(ICP_GN),
                         local_map_ptr_(new CloudData::CLOUD()),
                         global_map_ptr_(new CloudData::CLOUD()),
                         result_cloud_ptr_(new CloudData::CLOUD())
        {
            // 设置默认参数，以免类的使用者在匹配之前忘了设置参数
            icp_opti_.SetMaxCorrespondDistance(1);
            icp_opti_.SetMaxIterations(2);
            icp_opti_.SetTransformationEpsilon(0.5);
            
            // 设置NDT默认参数
            ndt_registration_.SetMaxIterations(35);
            ndt_registration_.SetTransformationEpsilon(0.01);
            ndt_registration_.SetStepSize(0.1);
            ndt_registration_.SetResolution(1.0);
            
            cloud_filter_.setLeafSize(1.5f,1.5f,1.5f);
            local_map_filter_.setLeafSize(1.0f,1.0f,1.0f);
            display_filter_.setLeafSize(1.0f,1.0f,1.0f);
            global_map_filter_.setLeafSize(0.5f,0.5f,0.5f);
        }
    
    // 输入当前帧扫描的点云图，对位姿进行更新
    Eigen::Matrix4f FrontEnd::Update(const CloudData& cloud_data)
    {
        // 参数初始化
        current_frame_.cloud_data.time = cloud_data.time;
        // 剔除无效点云：输入点云，输出点云，索引
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);
        // 下采样滤波
        CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD()); // 定义过滤后的点云指针
        cloud_filter_.setInputCloud(current_frame_.cloud_data.cloud_ptr);
        cloud_filter_.filter(*filtered_cloud_ptr);
        // 四个位姿
        static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
        static Eigen::Matrix4f last_pose = init_pose_;
        static Eigen::Matrix4f predict_pose = init_pose_;
        static Eigen::Matrix4f last_key_frame_pose = init_pose_;

        // 局部地图容器中没有关键帧，代表是第一帧数据
        // 此时把当前帧数据作为第一帧，并更新局部地图容器和全局地图容器
        if(local_map_frames_.size() == 0)
        {
            current_frame_.pose = init_pose_;
            UpdateNewFrame(current_frame_);
            return current_frame_.pose;
        }
        // 不是第一帧，就正常匹配
        bool match_success = false;
        if (registration_method_ == ICP_GN)
        {
            match_success = icp_opti_.Match(filtered_cloud_ptr, predict_pose, result_cloud_ptr_, current_frame_.pose);
            std::cout << "ICP fitness score: " << icp_opti_.GetFitnessScore() << std::endl;
        }
        else if (registration_method_ == NDT)
        {
            match_success = ndt_registration_.Match(filtered_cloud_ptr, predict_pose, result_cloud_ptr_, current_frame_.pose);
            std::cout << "NDT fitness score: " << ndt_registration_.GetFitnessScore() << std::endl;
        }
        
        if (!match_success)
        {
            LOG(WARNING) << "Registration failed, using predict pose";
            current_frame_.pose = predict_pose;
        }
        

        // 此处采用运动模型来做位姿预测（当然也可以用IMU）更新相邻两帧的相对运动
        step_pose = last_pose.inverse() * current_frame_.pose;
        predict_pose = current_frame_.pose * step_pose;
        last_pose = current_frame_.pose;

        // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
        if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
            fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
            fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > 2.0) 
        {
            UpdateNewFrame(current_frame_);
            last_key_frame_pose = current_frame_.pose;
        }

        return current_frame_.pose;
    }

    bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose)
    {
        init_pose_ = init_pose;
        return true;
    }

    bool FrontEnd::SetPredictPose(const Eigen::Matrix4f& predict_pose)
    {
        predict_pose_ = predict_pose;
        return true;
    }
    
    void FrontEnd::UpdateNewFrame(const Frame& new_key_frame)
    {
        Frame key_frame = new_key_frame;
        // 这一步的目的是为了把关键帧的点云保存下来。由于用的是共享指针，所以只是直接复制了一个指针而已
        // 此时无论你放多少个关键帧在容器里面，这些关键帧点云指针都是指向的同一个点云
        key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
        CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

        // 更新局部地图
        local_map_frames_.push_back(key_frame);
        while (local_map_frames_.size() > 20) // 局部地图滑窗队列大小固定为20
        {
            local_map_frames_.pop_front();
        }
        local_map_ptr_.reset(new CloudData::CLOUD()); // 更新局部地图指针
        
        // 遍历滑窗
        for(size_t i = 0; i < local_map_frames_.size(); ++i)
        {
            // 恢复下采样的点云数据，参数：输入，输出，估计的姿态
            pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, *transformed_cloud_ptr, local_map_frames_.at(i).pose);
            *local_map_ptr_ += *transformed_cloud_ptr;
        }
        has_new_local_map_ = true;

        // 更新匹配的目标点云
        if (local_map_frames_.size() < 10) // 如果局部地图数量少于10个，直接设置为目标点云
        {
            if (registration_method_ == ICP_GN)
            {
                icp_opti_.SetTargetCloud(local_map_ptr_);
            }
            else if (registration_method_ == NDT)
            {
                ndt_registration_.SetTargetCloud(local_map_ptr_);
            }
        }
        else // 否则，先对局部地图进行下采样，再加进去
        {
            CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
            local_map_filter_.setInputCloud(local_map_ptr_);
            local_map_filter_.filter(*filtered_local_map_ptr);
            
            if (registration_method_ == ICP_GN)
            {
                icp_opti_.SetTargetCloud(filtered_local_map_ptr);
            }
            else if (registration_method_ == NDT)
            {
                ndt_registration_.SetTargetCloud(filtered_local_map_ptr);
            }
        }
        
        // 更新全局地图
        UpdateGlobalMap(key_frame);
    }
    
    void FrontEnd::UpdateGlobalMap(const Frame& new_key_frame)
    {
        global_map_frames_.push_back(new_key_frame);
        
        // 每100帧更新一次全局地图
        if(global_map_frames_.size() % 100 != 0)
        {
            return;
        }
        
        global_map_ptr_.reset(new CloudData::CLOUD());
        CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
        
        for(size_t i = 0; i < global_map_frames_.size(); ++i)
        {
            pcl::transformPointCloud(*global_map_frames_.at(i).cloud_data.cloud_ptr, 
                                   *transformed_cloud_ptr, 
                                   global_map_frames_.at(i).pose);
            *global_map_ptr_ += *transformed_cloud_ptr;
        }
        
        // 对全局地图进行下采样以减少内存占用
        CloudData::CLOUD_PTR filtered_global_map_ptr(new CloudData::CLOUD());
        global_map_filter_.setInputCloud(global_map_ptr_);
        global_map_filter_.filter(*filtered_global_map_ptr);
        global_map_ptr_ = filtered_global_map_ptr;
        
        has_new_global_map_ = true;
        
        LOG(INFO) << "Global map updated with " << global_map_frames_.size() << " frames, "
                  << global_map_ptr_->size() << " points";
        
        // 自动保存全局地图为PCD文件
        std::string save_path = WORK_SPACE_PATH + "/slam_data/map/global_map.pcd";
        SaveGlobalMap(save_path);
        LOG(INFO) << "Global map saved to: " << save_path;
    }
    
    void FrontEnd::SetRegistrationMethod(RegistrationMethod method)
    {
        registration_method_ = method;
        LOG(INFO) << "Registration method set to: " << (method == ICP_GN ? "ICP_GN" : "NDT");
    }
    
    void FrontEnd::SaveGlobalMap(const std::string& file_path)
    {
        LOG(INFO) << "Attempting to save global map to: " << file_path;
        
        if (!global_map_ptr_)
        {
            LOG(ERROR) << "Global map pointer is null, cannot save";
            return;
        }
        
        if (global_map_ptr_->empty())
        {
            LOG(WARNING) << "Global map is empty, cannot save";
            return;
        }
        
        try
        {
            // 确保目录存在
            std::string dir_path = file_path.substr(0, file_path.find_last_of('/'));
            std::string mkdir_cmd = "mkdir -p " + dir_path;
            system(mkdir_cmd.c_str());
            
            LOG(INFO) << "Saving " << global_map_ptr_->size() << " points to PCD file";
            int result = pcl::io::savePCDFileBinary(file_path, *global_map_ptr_);
            
            if (result == 0)
            {
                LOG(INFO) << "Global map successfully saved to: " << file_path 
                          << " with " << global_map_ptr_->size() << " points";
            }
            else
            {
                LOG(ERROR) << "Failed to save PCD file, error code: " << result;
            }
        }
        catch (const std::exception& e)
        {
            LOG(ERROR) << "Exception while saving PCD file: " << e.what();
        }
    }
    
    void FrontEnd::SaveNDTRegistrationResult(const std::string& file_path)
    {
        LOG(INFO) << "Attempting to save NDT registration result to: " << file_path;
        
        if (!result_cloud_ptr_)
        {
            LOG(ERROR) << "NDT registration result pointer is null, cannot save";
            return;
        }
        
        if (result_cloud_ptr_->empty())
        {
            LOG(WARNING) << "NDT registration result is empty, cannot save";
            return;
        }
        
        try
        {
            // 确保目录存在
            std::string dir_path = file_path.substr(0, file_path.find_last_of('/'));
            std::string mkdir_cmd = "mkdir -p " + dir_path;
            system(mkdir_cmd.c_str());
            
            LOG(INFO) << "Saving " << result_cloud_ptr_->size() << " points to NDT result PCD file";
            int result = pcl::io::savePCDFileBinary(file_path, *result_cloud_ptr_);
            
            if (result == 0)
            {
                LOG(INFO) << "NDT registration result successfully saved to: " << file_path 
                          << " with " << result_cloud_ptr_->size() << " points";
            }
            else
            {
                LOG(ERROR) << "Failed to save NDT result PCD file, error code: " << result;
            }
        }
        catch (const std::exception& e)
        {
            LOG(ERROR) << "Exception while saving NDT result PCD file: " << e.what();
        }
    }
    
    bool FrontEnd::GetNDTRegistrationResult(CloudData::CLOUD_PTR& ndt_result_ptr)
    {
        if (!result_cloud_ptr_ || result_cloud_ptr_->empty())
        {
            return false;
        }
        
        display_filter_.setInputCloud(result_cloud_ptr_);
        display_filter_.filter(*ndt_result_ptr);
        return true;
    }
    
    void FrontEnd::ResetGlobalMap()
    {
        global_map_frames_.clear();
        global_map_ptr_.reset(new CloudData::CLOUD());
        has_new_global_map_ = false;
        LOG(INFO) << "Global map reset";
    }
    
    size_t FrontEnd::GetGlobalMapSize() const
    {
        return global_map_frames_.size();
    }

    bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr)
    {
        if (has_new_local_map_)
        {
            display_filter_.setInputCloud(local_map_ptr_);
            display_filter_.filter(*local_map_ptr);
            return true;
        }
        return false;
    }

    bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr)
    {
        if (has_new_global_map_)
        {
            display_filter_.setInputCloud(global_map_ptr_);
            display_filter_.filter(*global_map_ptr);
            return true;
        }
        return false;  
    }

    bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr)
    {
        display_filter_.setInputCloud(result_cloud_ptr_);
        display_filter_.filter(*current_scan_ptr);
        return true;
    }
}