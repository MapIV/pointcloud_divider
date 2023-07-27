# pointcloud_divider

(Updated 2023/03/01)

Dividing large PCD files into 2D grids.

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

  ```
  cd hoge_ws
  ./src/pointcloud_divider/scripts/pointcloud_divider.sh <INPUT_DIR> <OUTPUT_DIR> <PREFIX> <CONFIG>
  ```

  * Select indivisual files

  ```
  cd hoge_ws
  ./src/pointcloud_divider/scripts/divider_core.sh <PCD_0> ... <PCD_N> <OUTPUT_DIR> <PREFIX> <CONFIG>
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

## LICENSE

This repository is under [BSD-3-Clauses](LICENSE) license
