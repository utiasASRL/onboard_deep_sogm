from distutils.core import setup, Extension
import numpy.distutils.misc_util

# Adding Eigen to project
# ***********************

EIGEN_INCLUDE = "../src/Eigen"

# Adding sources of the project
# *****************************

SOURCES = ["../src/cloud/cloud.cpp",
           "../src/cloud/points.cpp",
           "../src/npm_ply/ply_file_in.cc",
           "../src/npm_ply/ply_file_out.cc",
           "../src/icp/icp.cpp",
           "wrap_icp.cpp"]

module = Extension(name="icp",
                   sources=SOURCES,
                   include_dirs=[EIGEN_INCLUDE],
                   extra_compile_args=['-std=c++11',
                                       '-D_GLIBCXX_USE_CXX11_ABI=0'])


setup(ext_modules=[module], include_dirs=numpy.distutils.misc_util.get_numpy_include_dirs())








