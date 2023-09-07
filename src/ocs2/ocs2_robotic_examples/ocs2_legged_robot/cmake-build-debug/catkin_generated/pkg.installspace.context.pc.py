# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3;/usr/include;/opt/openrobots/lib/pkgconfig/../../include;/opt/openrobots/include".split(';') if "${prefix}/include;/usr/include/eigen3;/usr/include;/opt/openrobots/lib/pkgconfig/../../include;/opt/openrobots/include" != "" else []
PROJECT_CATKIN_DEPENDS = "ocs2_core;ocs2_oc;ocs2_ddp;ocs2_mpc;ocs2_sqp;ocs2_ipm;ocs2_robotic_tools;ocs2_pinocchio_interface;ocs2_centroidal_model;ocs2_robotic_assets".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-locs2_legged_robot;/usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0;-lpinocchio;-lboost_filesystem;-lboost_serialization;-lboost_system;-lurdfdom_sensor;-lurdfdom_model_state;-lurdfdom_model;-lurdfdom_world;-lconsole_bridge;-lhpp-fcl;-loctomap;-loctomath".split(';') if "-locs2_legged_robot;/usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0;-lpinocchio;-lboost_filesystem;-lboost_serialization;-lboost_system;-lurdfdom_sensor;-lurdfdom_model_state;-lurdfdom_model;-lurdfdom_world;-lconsole_bridge;-lhpp-fcl;-loctomap;-loctomath" != "" else []
PROJECT_NAME = "ocs2_legged_robot"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.1"
