# pointcloud_divider

(Updated 2023/03/27)

Dividing large PCD files into 2D grids.

**Currently, only pcl::PointXYZI is supported. Any PCD will be loaded as pcl::PointXYZI.**

## Installation

```bash
$ git clone https://github.com/MapIV/pointcloud_divider.git
$ cd pointcloud_divider
$ mkdir build
$ cd build
$ cmake ..
$ make
```

## Usage

  * Select directory, process all files found with `find $INPUT_DIR -name "*.pcd"`.

  ```bash
  $ ./scripts/pointcloud_divider.sh <INPUT_DIR> <OUTPUT_DIR> <PREFIX> <CONFIG>
  ```

  * Select individual files

  ```bash
  $ ./scripts/divider_core.sh <PCD_0> ... <PCD_N> <OUTPUT_DIR> <PREFIX> <CONFIG>
  ```

  | Name       | Description                                  |
  | -------    | -------------------------                    |
  | INPUT_DIR  | Directory that contains all PCD files        |
  | PCD_N      | Input PCD file name                          |
  | OUTPUT_DIR | Output directory name                        |
  | PREFIX     | Prefix of output PCD file name               |
  | CONFIG     | Config file ([default](config/default.yaml)) |

## Parameter

  * use_large_grid

    Pack output PCD files in larger grid directory

  * merge_pcds

    Merge all grid into a single PCD

  * leaf_size

    leaf_size of voxel grid filter [m]

  * grid_size_x(/y)

    Size of grid [m]

## Metadata YAML Format

The metadata file should be named `metadata.yaml`. It contains the following fields:

- `x_resolution`: The resolution along the X-axis.
- `y_resolution`: The resolution along the Y-axis.

Additionally, the file contains entries for individual point cloud files (`.pcd` files) and their corresponding grid coordinates. The key is the file name, and the value is a list containing the X and Y coordinates of the lower-left corner of the grid cell associated with that file. The grid cell's boundaries can be calculated using the `x_resolution` and `y_resolution` values.

For example:

```yaml
x_resolution: 100.0
y_resolution: 150.0
A.pcd: [1200, 2500] # -> 1200 <= x <= 1300, 2500 <= y <= 2650
B.pcd: [1300, 2500] # -> 1300 <= x <= 1400, 2500 <= y <= 2650
C.pcd: [1200, 2650] # -> 1200 <= x <= 1300, 2650 <= y <= 2800
D.pcd: [1400, 2650] # -> 1400 <= x <= 1500, 2650 <= y <= 2800
...
```

## LICENSE

This repository is under [BSD-3-Clauses](LICENSE) license
