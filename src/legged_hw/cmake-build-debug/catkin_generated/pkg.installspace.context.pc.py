# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;legged_common;controller_manager;urdf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-llegged_hw".split(';') if "-llegged_hw" != "" else []
PROJECT_NAME = "legged_hw"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
