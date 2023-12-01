#ifndef POINTCLOUD_DIVIDER_HPP
#define POINTCLOUD_DIVIDER_HPP

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <unordered_set>

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

struct GridInfo
{
  int x, y, gx, gy;
  GridInfo() : x(0), y(0), gx(0), gy(0)
  {
  }
  GridInfo(int x, int y) : x(x), y(y)
  {
  }
  GridInfo(int x, int y, int gx, int gy) : x(x), y(y), gx(gx), gy(gy)
  {
  }
  friend bool operator==(const GridInfo& one, const GridInfo& other)
  {
    return one.x == other.x && one.y == other.y;
  }
  friend bool operator!=(const GridInfo& one, const GridInfo& other)
  {
    return !(one == other);
  }
  friend std::ostream& operator<<(std::ostream& os, const GridInfo& g)
  {
    os << g.x << ' ' << g.y;
    return os;
  }
};

// This is for unordered_map and unodered_set
namespace std
{
template <>
struct hash<GridInfo>
{
public:
  size_t operator()(const GridInfo& grid) const
  {
    std::size_t seed = 0;
    seed ^= std::hash<int>{}(grid.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>{}(grid.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};
}  // namespace std

template <class PointT>
class PointCloudDivider
{
public:
  PointCloudDivider()
  {
  }
  ~PointCloudDivider()
  {
  }

  std::pair<double, double> getGridSize() const
  {
    return std::pair<double, double>(grid_size_x_, grid_size_y_);
  }

  void run(std::vector<std::string> pcd_names, std::string output_dir, std::string file_prefix, std::string config);

  void run(const typename pcl::PointCloud<PointT>::Ptr& cloud, std::string output_dir, std::string file_prefix,
           std::string config);

  std::string makeFileName(const GridInfo& grid) const;

  GridInfo pointToGrid(const Eigen::Vector3f& pos) const;

  std::pair<int, int> gridToCenter(const GridInfo& grid) const;

  void setUseLargeGrid(const bool use_large_grid)
  {
    use_large_grid_ = use_large_grid;
  }

private:
  typename pcl::PointCloud<PointT>::Ptr cloud_ptr_;
  typename pcl::PointCloud<PointT>::Ptr merged_ptr_;
  std::string output_dir_, file_prefix_, config_file_;

  std::unordered_set<GridInfo> grid_set_;

  // Params from yaml
  bool use_large_grid_ = false;
  bool merge_pcds_ = false;
  double leaf_size_ = 0.1;
  double grid_size_x_ = 100;
  double grid_size_y_ = 100;
  double g_grid_size_x_ = grid_size_x_ * 10;
  double g_grid_size_y_ = grid_size_y_ * 10;

  std::unordered_map<GridInfo, pcl::PointCloud<PointT>> grid_to_cloud;

  typename pcl::PointCloud<PointT>::Ptr loadPCD(const std::string& pcd_name);
  void savePCD(const std::string& pcd_name, const pcl::PointCloud<PointT>& cloud);
  void saveMergedPCD();
  void dividePointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_ptr);
  void saveGridPCD();
  void paramInitialize();
  void saveGridInfoToYAML(const std::string& yaml_file_path);
  void checkOutputDirectoryValidity() const;
};

#endif
