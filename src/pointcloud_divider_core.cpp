#include <pointcloud_divider/pointcloud_divider.hpp>

#include <experimental/filesystem>

#include <pcl/filters/voxel_grid.h>

namespace fs = std::experimental::filesystem;

template <class PointT>
std::unordered_set<GridInfo> PointCloudDivider<PointT>::run(const typename pcl::PointCloud<PointT>::Ptr& cloud_ptr,
                                                            std::string output_dir, std::string file_prefix,
                                                            std::string config)
{
  output_dir_ = output_dir;
  file_prefix_ = file_prefix;
  config_file_ = config;

  grid_set_.clear();

  paramInitialize();
  dividePointCloud(cloud_ptr);
  saveGridPCD();
  return grid_set_;
}

template <class PointT>
std::unordered_set<GridInfo> PointCloudDivider<PointT>::run(std::vector<std::string> pcd_names, std::string output_dir,
                                                            std::string file_prefix, std::string config)
{
  output_dir_ = output_dir;
  file_prefix_ = file_prefix;
  config_file_ = config;

  grid_set_.clear();
  paramInitialize();

  for (const std::string& pcd_name : pcd_names)
  {
    std::cout << pcd_name << std::endl;

    typename pcl::PointCloud<PointT>::Ptr cloud_ptr = loadPCD(pcd_name);
    dividePointCloud(cloud_ptr);
    saveGridPCD();
  }
  return grid_set_;
}

template <class PointT>
typename pcl::PointCloud<PointT>::Ptr PointCloudDivider<PointT>::loadPCD(const std::string& pcd_name)
{
  typename pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile(pcd_name, *cloud_ptr) == -1)
  {
    std::cerr << "Error: Cannot load PCD: " << pcd_name << std::endl;
    exit(1);
  }
  return cloud_ptr;
}

template <class PointT>
GridInfo PointCloudDivider<PointT>::pointToGrid(const Eigen::Vector3f& pos) const
{
  int x_id = static_cast<int>(std::floor(pos.x() / grid_size_x_) * grid_size_x_);
  int y_id = static_cast<int>(std::floor(pos.y() / grid_size_y_) * grid_size_y_);
  int gx_id = static_cast<int>(std::floor(pos.x() / g_grid_size_x_) * g_grid_size_x_);
  int gy_id = static_cast<int>(std::floor(pos.y() / g_grid_size_y_) * g_grid_size_y_);
  return GridInfo(x_id, y_id, gx_id, gy_id);
}

template <class PointT>
std::pair<int, int> PointCloudDivider<PointT>::gridToCenter(const GridInfo& grid) const
{
  int x = static_cast<int>(grid.x + grid_size_x_ * 0.5);
  int y = static_cast<int>(grid.y + grid_size_y_ * 0.5);
  return { x, y };
}

template <class PointT>
void PointCloudDivider<PointT>::dividePointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_ptr)
{
  for (const PointT p : *cloud_ptr)
  {
    GridInfo tmp = pointToGrid(p.getVector3fMap());
    grid_to_cloud[tmp].push_back(p);
  }
}

template <class PointT>
std::string PointCloudDivider<PointT>::makeFileName(const GridInfo& grid) const
{
  std::string file_name = output_dir_;
  if (use_large_grid_)
  {
    file_name = file_name + "/" + std::to_string(grid.gx) + "_" + std::to_string(grid.gy) + "/";
    if (!fs::exists(file_name))
      fs::create_directory(file_name);
  }

  file_name = file_name + file_prefix_ + "_";
  file_name = file_name + std::to_string(grid.x) + "_" + std::to_string(grid.y) + ".pcd";

  return file_name;
}

template <class PointT>
void PointCloudDivider<PointT>::saveGridPCD()
{
  for (std::pair<GridInfo, pcl::PointCloud<PointT>> e : grid_to_cloud)
  {
    grid_set_.insert(e.first);
    const std::string file_name = makeFileName(e.first);
    if (fs::exists(file_name))
    {
      pcl::PointCloud<PointT> tmp;
      if (pcl::io::loadPCDFile(file_name, tmp) == -1)
      {
        std::cerr << "Error: Cannot save PCD: " << file_name << std::endl;
        exit(1);
      }
      e.second += tmp;
    }

    if (leaf_size_ > 0)
    {
      pcl::PointCloud<PointT> filtered;
      if (leaf_size_ != 0)
      {
        pcl::VoxelGrid<PointT> vgf;
        typename pcl::PointCloud<PointT>::Ptr tmp_ptr(new pcl::PointCloud<PointT>);
        *tmp_ptr = e.second;
        vgf.setInputCloud(tmp_ptr);
        vgf.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        vgf.filter(filtered);
        e.second = filtered;
      }
    }

    if (pcl::io::savePCDFileBinary(file_name, e.second) == -1)
    {
      std::cerr << "Error: Cannot save PCD: " << file_name << std::endl;
      exit(1);
    }
  }

  grid_to_cloud.clear();
}

template <class PointT>
void PointCloudDivider<PointT>::paramInitialize()
{
  try
  {
    YAML::Node conf = YAML::LoadFile(config_file_)["pointcloud_divider"];
    use_large_grid_ = conf["use_large_grid"].as<bool>();
    leaf_size_ = conf["leaf_size"].as<double>();
    grid_size_x_ = conf["grid_size_x"].as<double>();
    grid_size_y_ = conf["grid_size_y"].as<double>();
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "YAML Error: " << e.what() << std::endl;
    exit(1);
  }

  g_grid_size_x_ = grid_size_x_ * 10;
  g_grid_size_y_ = grid_size_y_ * 10;
}

template class PointCloudDivider<pcl::PointXYZ>;
template class PointCloudDivider<pcl::PointXYZI>;
template class PointCloudDivider<pcl::PointXYZINormal>;
template class PointCloudDivider<pcl::PointXYZRGB>;
template class PointCloudDivider<pcl::PointNormal>;
template class PointCloudDivider<PointXYZISC>;
template class PointCloudDivider<PointXYZIRGBSC>;