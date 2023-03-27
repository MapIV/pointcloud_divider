#include <pointcloud_divider/pointcloud_divider.hpp>

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

  // Currently only PointXYZI is supported
  PointCloudDivider<pcl::PointXYZI> divider;
  divider.run(pcd_name, output_dir, prefix, config);

  return 0;
}
