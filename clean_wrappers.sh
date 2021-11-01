#!/bin/bash

cd src/deep_sogm/deep_sogm/cpp_wrappers

# Compile cpp icp normals
cd cpp_pointmap
rm -r build
rm *.so
cd ..

# Compile cpp subsampling
cd cpp_subsampling
rm -r build
rm *.so
cd ..

# Compile cpp neighbors
cd cpp_neighbors
rm -r build
rm *.so
cd ..

# Compile cpp polar normals
cd cpp_polar_normals
rm -r build
rm *.so
cd ..

# Compile cpp icp normals
cd cpp_icp
rm -r build
rm *.so
cd ..

# Compile cpp region growing
# cd cpp_region_growing
# python3 setup.py build_ext --inplace
# cd ..

# Compile cpp icp normals
cd cpp_slam
rm -r build
rm *.so
cd ..

# Compile cpp icp normals
cd cpp_lidar_utils
rm -r build
rm *.so
cd ..