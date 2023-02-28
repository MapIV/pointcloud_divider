# pointcloud_divider

(Updated 2023/02/28)

Dividing large PCD files into 2D grids.

## Installation

  ```
  mkdir -p hoge_ws/src
  cd hoge_ws/src
  git clone git@github.com:MapIV/map4_point_type.git
  git clone git@github.com:MapIV/pointcloud_divider.git
  rosdep update
  rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
  catkin_make -DCMAKE_BUILD_TYPE=Release
  ```

  ## Usage

  ```
  ./ros/devel/lib/pointcloud_divider/pointcloud_divider <NUM_PCD> <PCD_0> ... <PCD_N> <OUTPUT_DIR> <PREFIX> <CONFIG>
  ```

  | Name       | Description                       |
  | -------    | -------------------------         |
  | NUM_PCD    | Number of input PCD files         |
  | PCD_N      | Input PCD file name               |
  | OUTPUT_DIR | Output directory name             |
  | PREFIX     | Prefix of output PCD file name    |
  | CONFIG     | Config file (config/default.yaml) |

## LICENSE

This repository is under [BSD-3-Clauses](LICENSE) license
