# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;legged_common;legged_interface;legged_wbc;legged_estimation;controller_interface;ocs2_legged_robot_ros;ocs2_self_collision_visualization;angles".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-llegged_controllers".split(';') if "-llegged_controllers" != "" else []
PROJECT_NAME = "legged_controllers"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
