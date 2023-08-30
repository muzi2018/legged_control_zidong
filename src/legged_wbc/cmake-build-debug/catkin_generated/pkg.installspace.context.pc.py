# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/opt/openrobots/lib/pkgconfig/../../include;/opt/openrobots/include;/usr/include/eigen3".split(';') if "${prefix}/include;/opt/openrobots/lib/pkgconfig/../../include;/opt/openrobots/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;ocs2_legged_robot".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-llegged_wbc;-lpinocchio;-lboost_filesystem;-lboost_serialization;-lboost_system;-lurdfdom_sensor;-lurdfdom_model_state;-lurdfdom_model;-lurdfdom_world;-lconsole_bridge;-lhpp-fcl;-loctomap;-loctomath".split(';') if "-llegged_wbc;-lpinocchio;-lboost_filesystem;-lboost_serialization;-lboost_system;-lurdfdom_sensor;-lurdfdom_model_state;-lurdfdom_model;-lurdfdom_world;-lconsole_bridge;-lhpp-fcl;-loctomap;-loctomath" != "" else []
PROJECT_NAME = "legged_wbc"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
