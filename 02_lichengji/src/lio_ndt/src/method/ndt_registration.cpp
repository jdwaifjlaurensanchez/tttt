#include "lio_ndt/method/ndt_registration.h"
#include <pcl/common/transforms.h>

NDTRegistration::NDTRegistration()
    : target_cloud_ptr_(nullptr), source_cloud_ptr_(nullptr), has_converged_(false)
{
    // 设置NDT默认参数
    ndt_.setTransformationEpsilon(0.01);
    ndt_.setStepSize(0.1);
    ndt_.setResolution(1.0);
    ndt_.setMaximumIterations(35);
}

bool NDTRegistration::SetTargetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud_ptr)
{
    if (!target_cloud_ptr || target_cloud_ptr->empty())
    {
        return false;
    }
    
    target_cloud_ptr_ = target_cloud_ptr;
    ndt_.setInputTarget(target_cloud_ptr_);
    return true;
}

bool NDTRegistration::Match(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud_ptr,
                           const Eigen::Matrix4f &predict_pose,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_source_cloud_ptr,
                           Eigen::Matrix4f &result_pose)
{
    if (!source_cloud_ptr || source_cloud_ptr->empty())
    {
        return false;
    }
    
    if (!target_cloud_ptr_ || target_cloud_ptr_->empty())
    {
        return false;
    }
    
    source_cloud_ptr_ = source_cloud_ptr;
    ndt_.setInputSource(source_cloud_ptr_);
    
    // 执行NDT配准
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt_.align(*output_cloud, predict_pose);
    
    has_converged_ = ndt_.hasConverged();
    
    if (has_converged_)
    {
        final_transformation_ = ndt_.getFinalTransformation();
        result_pose = final_transformation_;
        
        // 应用变换到源点云
        pcl::transformPointCloud(*source_cloud_ptr_, *transformed_source_cloud_ptr, result_pose);
        
        return true;
    }
    
    return false;
}

float NDTRegistration::GetFitnessScore(float max_range)
{
    if (!has_converged_)
    {
        return std::numeric_limits<float>::max();
    }
    
    return ndt_.getFitnessScore(static_cast<double>(max_range));
}

void NDTRegistration::SetMaxIterations(unsigned int iter)
{
    ndt_.setMaximumIterations(iter);
}

void NDTRegistration::SetTransformationEpsilon(float transformation_epsilon)
{
    ndt_.setTransformationEpsilon(transformation_epsilon);
}

void NDTRegistration::SetStepSize(float step_size)
{
    ndt_.setStepSize(step_size);
}

void NDTRegistration::SetResolution(float resolution)
{
    ndt_.setResolution(resolution);
}

bool NDTRegistration::HasConverged() const
{
    return has_converged_;
}