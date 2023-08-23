cmake_minimum_required(VERSION 3.10)
project(humanoid_description)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        )

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
)

#############
## Install ##
#############

# Mark resource files for installation
install(
        DIRECTORY meshes_zidong_obj urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
