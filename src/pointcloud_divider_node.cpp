#include <pointcloud_divider/pointcloud_divider.hpp>

#include <map4_point_type/type_detector.hpp>

namespace mpe = map4_pcl_extensions;

void printInvalidArguments()
{
  std::cerr << "Error: Invalid Arugments" << std::endl;
  exit(1);
}

int main(int argc, char* argv[])
{
  int n_pcd;
  std::vector<std::string> pcd_name;
  std::string output_dir, prefix, config;

  if (argc <= 0)
  {
    printInvalidArguments();
  }

  n_pcd = std::stoi(argv[1]);
  if (argc == 5 + n_pcd)
  {
    for (int pcd_id = 0; pcd_id < n_pcd; pcd_id++)
    {
      pcd_name.push_back(argv[2 + pcd_id]);
    }
    output_dir = argv[n_pcd + 2];
    prefix = argv[n_pcd + 3];
    config = argv[n_pcd + 4];
  }
  else
  {
    printInvalidArguments();
  }

  mpe::PointType input_point_type = mpe::detectType(pcd_name[0]);
  if (static_cast<int>(input_point_type) < 0)  // Double XYZ
  {
    std::cerr << "\033[31;1mError: DXYZ Point Type is not supported: " << static_cast<int>(input_point_type) << "\033[m"
              << std::endl;
    exit(4);
  }
  else
  {
    // Double to Float
    if (input_point_type == mpe::PointType::PointXYZ)
    {
      PointCloudDivider<pcl::PointXYZ> divider;
      divider.run(pcd_name, output_dir, prefix, config);
    }
    else if (input_point_type == mpe::PointType::PointXYZI)
    {
      PointCloudDivider<pcl::PointXYZI> divider;
      divider.run(pcd_name, output_dir, prefix, config);
    }
    else if (input_point_type == mpe::PointType::PointXYZRGB)
    {
      PointCloudDivider<pcl::PointXYZRGB> divider;
      divider.run(pcd_name, output_dir, prefix, config);
    }
    else if (input_point_type == mpe::PointType::PointXYZISC)
    {
      PointCloudDivider<PointXYZISC> divider;
      divider.run(pcd_name, output_dir, prefix, config);
    }
    else if (input_point_type == mpe::PointType::PointXYZIRGBSC)
    {
      PointCloudDivider<PointXYZIRGBSC> divider;
      divider.run(pcd_name, output_dir, prefix, config);
    }
  }

  return 0;
}
