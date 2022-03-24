cmake_minimum_required(VERSION 3.14)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)

project(ba_demo)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules")


# Find Eigen3. If it defines the target, this is used. If not,
# fall back to the using the module form.
# See https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html for details
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV 3.4.2 EXACT REQUIRED)

if (TARGET Eigen3::Eigen)
  set(G2O_EIGEN3_EIGEN_TARGET Eigen3::Eigen)
else()
  include_directories(${EIGEN3_INCLUDE_DIR})
endif ()


include_directories(ba_demo 
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${LIBAVL_INCLUDE_DIR}
  )


add_executable(ba_demo
  ba_demo.cpp
  DataReader.cpp
)

link_directories(/usr/local/lib)

target_link_libraries(ba_demo 
  ${OpenCV_LIBS}
  g2o_core g2o_types_sba g2o_solver_dense 
    g2o_cli.so                   g2o_hierarchical.so  g2o_opengl_helper.so   g2o_solver_csparse.so  g2o_solver_slam2d_linear.so   g2o_types_icp.so      g2o_types_slam2d_addons.so
    g2o_core.so                  g2o_incremental.so   g2o_parser.so          g2o_solver_dense.so    g2o_solver_structure_only.so  g2o_types_sba.so      g2o_types_slam2d.so
    g2o_csparse_extension.so     g2o_interactive.so   g2o_simulator.so       g2o_solver_eigen.so    g2o_stuff.so                  g2o_types_sclam2d.so  g2o_types_slam3d_addons.so
    g2o_ext_freeglut_minimal.so  g2o_interface.so     g2o_solver_cholmod.so  g2o_solver_pcg.so      g2o_types_data.so             g2o_types_sim3.so     g2o_types_slam3d.so
)

MESSAGE("OpenCV_INCLUDE_DIRS " ${OpenCV_INCLUDE_DIRS})
MESSAGE("OpenCV_LIBRARY_DIRS " ${OpenCV_LIBRARY_DIRS})
MESSAGE("OpenCV_LIBS " ${OpenCV_LIBS})
MESSAGE("CMAKE_SOURCE_DIR " ${CMAKE_SOURCE_DIR})
MESSAGE("CMAKE_CURRENT_SOURCE_DIR " ${CMAKE_CURRENT_SOURCE_DIR})


if(CHOLMOD_FOUND)
  target_link_libraries(ba_demolver_cholmod)
else()
  target_link_libraries(ba_demo -lg2o_solver_eigen)
endif()
