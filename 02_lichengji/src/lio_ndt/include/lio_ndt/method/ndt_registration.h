#ifndef NDT_REGISTRATION_H
#define NDT_REGISTRATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <eigen3/Eigen/Core>

class NDTRegistration 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    NDTRegistration();

    /**
     * @brief 设置目标点云
     * @param target_cloud_ptr 目标点云指针
     * @return 是否设置成功
     */
    bool SetTargetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud_ptr);

    /**
     * @brief 执行NDT配准
     * @param source_cloud_ptr 源点云指针
     * @param predict_pose 预测位姿
     * @param transformed_source_cloud_ptr 变换后的源点云指针
     * @param result_pose 结果位姿
     * @return 是否配准成功
     */
    bool Match(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud_ptr,
               const Eigen::Matrix4f &predict_pose,
               pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_source_cloud_ptr,
               Eigen::Matrix4f &result_pose);

    /**
     * @brief 获取适应度分数
     * @param max_range 最大范围
     * @return 适应度分数
     */
    float GetFitnessScore(float max_range = std::numeric_limits<float>::max());

    /**
     * @brief 设置最大迭代次数
     * @param iter 迭代次数
     */
    void SetMaxIterations(unsigned int iter);

    /**
     * @brief 设置变换收敛阈值
     * @param transformation_epsilon 变换收敛阈值
     */
    void SetTransformationEpsilon(float transformation_epsilon);

    /**
     * @brief 设置步长大小
     * @param step_size 步长大小
     */
    void SetStepSize(float step_size);

    /**
     * @brief 设置分辨率
     * @param resolution 分辨率
     */
    void SetResolution(float resolution);

    /**
     * @brief 检查是否收敛
     * @return 是否收敛
     */
    bool HasConverged() const;

private:
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr_;
    Eigen::Matrix4f final_transformation_;
    bool has_converged_;
};

#endif // NDT_REGISTRATION_H