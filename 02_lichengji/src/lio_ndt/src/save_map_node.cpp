#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <lio_ndt/global_defination/global_defination.h>

class MapSaver
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber global_map_sub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_map_;
    bool map_received_;
    
public:
    MapSaver() : nh_("~"), accumulated_map_(new pcl::PointCloud<pcl::PointXYZ>), map_received_(false)
    {
        global_map_sub_ = nh_.subscribe("/global_map", 1, &MapSaver::globalMapCallback, this);
        ROS_INFO("Map saver node started. Waiting for global map data...");
    }
    
    void globalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *temp_cloud);
        
        if (!temp_cloud->empty())
        {
            *accumulated_map_ += *temp_cloud;
            map_received_ = true;
            ROS_INFO("Received global map with %zu points, total accumulated: %zu points", 
                     temp_cloud->size(), accumulated_map_->size());
        }
    }
    
    void saveMap()
    {
        if (!map_received_ || accumulated_map_->empty())
        {
            ROS_WARN("No map data received or map is empty!");
            return;
        }
        
        std::string save_path = lio_ndt::WORK_SPACE_PATH + "/slam_data/map/global_map.pcd";
        
        // 确保目录存在
        std::string mkdir_cmd = "mkdir -p " + lio_ndt::WORK_SPACE_PATH + "/slam_data/map";
        system(mkdir_cmd.c_str());
        
        try
        {
            int result = pcl::io::savePCDFileBinary(save_path, *accumulated_map_);
            if (result == 0)
            {
                ROS_INFO("Successfully saved global map to: %s with %zu points", 
                         save_path.c_str(), accumulated_map_->size());
            }
            else
            {
                ROS_ERROR("Failed to save PCD file, error code: %d", result);
            }
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Exception while saving PCD file: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_map_node");
    
    MapSaver map_saver;
    
    ROS_INFO("Collecting map data for 10 seconds...");
    ros::Duration(10.0).sleep();
    
    ros::spinOnce();
    
    map_saver.saveMap();
    
    ROS_INFO("Map saving completed. Node shutting down.");
    
    return 0;
}