cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(grid_service VERSION 0.1.0)


find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})
set(CMAKE_BUILD_TYPE Release)

# Add mono to invoke gk_ma.
find_package(PkgConfig)
pkg_check_modules(MONO mono-2 REQUIRED)
include_directories(${MONO_INCLUDE_DIRS})

# Download gk_ma

set(GKMA_INCLUDE_DIR ${CMAKE_BINARY_DIR}/gk_ma-prefix/src/gk_ma)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib)
message(CMAKE_LIBRARY_OUTPUT_DIRECTORY="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
include(ExternalProject)
ExternalProject_Add(
  gk_ma
  URL http://www.cs.nott.ac.uk/~pszdk/gtsp_ma_source_codes.zip
  URL https://polybox.ethz.ch/index.php/s/H4NXeaNPWo6VBrf/download
  DOWNLOAD_NAME gtsp_ma_source_codes.zip
  URL_MD5 765fad8e3746fa3dd9b81be0afb34d35
  PATCH_COMMAND
    COMMAND patch GkMa/OurHeuristic/Algorithm.cs ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/Algorithm.patch
    COMMAND patch NativeHelper/ClusterOptimisation.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/ClusterOptimisationCpp.patch
    COMMAND patch GkMa/OurHeuristic/Types/Generation.cs ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/Generation.patch
    COMMAND patch GkMa/OurHeuristic/GeneticAlgorithm.cs ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/GeneticAlgorithm.patch
    COMMAND patch GkMa/Helper.cs ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/Helper.patch
    COMMAND patch NativeHelper/ClusterOptimisation.h ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/ClusterOptimisationH.patch
    COMMAND patch NativeHelper/ImprovementManager.h ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/ImprovementManagerH.patch
    COMMAND patch NativeHelper/Insert.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/InsertCpp.patch
    COMMAND patch NativeHelper/Insert.h ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/InsertH.patch
    COMMAND patch NativeHelper/NativeHelper.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/NativeHelperCpp.patch
    COMMAND patch NativeHelper/Swap.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/SwapCpp.patch
    COMMAND patch NativeHelper/NativeHelper.h ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/NativeHelperH.patch
    COMMAND patch GkMa/OurHeuristic/NativeHelper.cs ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/NativeHelper.patch
    COMMAND patch GkMa/OurSolver.cs ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/OurSolver.patch
    COMMAND patch GkMa/OurHeuristic/Types/Permutation.cs ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/Permutation.patch
    COMMAND patch GkMa/Program.cs ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/Program.patch
    COMMAND patch GkMa/Solver.cs ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/Solver.patch
    COMMAND patch GkMa/Loader/Task.cs ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/Task.patch
    COMMAND patch GkMa/OurHeuristic/Types/Tour.cs ${CMAKE_CURRENT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/Tour.patch
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND
    COMMAND cp ${PROJECT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/MakefileCpp ./MakefileCpp
    COMMAND cp ${PROJECT_SOURCE_DIR}/src/polygon_coverage_solvers/patches/MakefileCs ./MakefileCs
  BUILD_COMMAND
    COMMAND $(MAKE) -f MakefileCs BUILD_PATH="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
    COMMAND $(MAKE) -f MakefileCpp BUILD_PATH="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
  INSTALL_COMMAND
    COMMAND ${CMAKE_COMMAND} -E create_symlink /usr/lib/libmono-native.so ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/System.Native
)

# Download GTSP test instances.
include(ExternalProject)
ExternalProject_Add(
  gtsp_instances
  URL http://www.cs.nott.ac.uk/~dxk/gtsplib/InstancesBinary.zip
  URL https://polybox.ethz.ch/index.php/s/51iqurpOOQ5cVaJ/download
  DOWNLOAD_NAME InstancesBinary.zip
  URL_MD5 255831bd47de71df8419a54741f0a6be
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

file(GLOB SRC_FILES "src/*.cpp")


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

#enable_testing()

add_executable(grid_service
    ${PB_FILES}
    main.cpp

    src/polygon_coverage_geometry/src/bcd.cc
    src/polygon_coverage_geometry/src/boolean.cc
    src/polygon_coverage_geometry/src/cgal_comm.cc
    src/polygon_coverage_geometry/src/decomposition.cc
    src/polygon_coverage_geometry/src/offset.cc
    src/polygon_coverage_geometry/src/sweep.cc
    src/polygon_coverage_geometry/src/tcd.cc
    src/polygon_coverage_geometry/src/triangulation.cc
    src/polygon_coverage_geometry/src/visibility_graph.cc
    src/polygon_coverage_geometry/src/visibility_polygon.cc
    src/polygon_coverage_geometry/src/weakly_monotone.cc

    src/polygon_coverage_planners/src/cost_functions/path_cost_functions.cc
    src/polygon_coverage_planners/src/graphs/gtspp_product_graph.cc
    src/polygon_coverage_planners/src/graphs/sweep_plan_graph.cc
    src/polygon_coverage_planners/src/planners/polygon_stripmap_planner_exact_preprocessed.cc
    src/polygon_coverage_planners/src/planners/polygon_stripmap_planner_exact.cc
    src/polygon_coverage_planners/src/planners/polygon_stripmap_planner.cc
    src/polygon_coverage_planners/src/timing.cc 

    src/polygon_coverage_solvers/src/boolean_lattice.cc
    src/polygon_coverage_solvers/src/combinatorics.cc
    src/polygon_coverage_solvers/src/gk_ma.cc
    src/polygon_planner_base.cc
    src/shortest_path_planner.cc
    src/coverage_planner.cc

)



target_include_directories(grid_service PUBLIC
    ./include
    ./src/polygon_coverage_planners/include
    ./src/polygon_coverage_geometry/include
    ./src/polygon_coverage_solvers/include
)

target_link_libraries(grid_service
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF}
    ${CGAL_LIBRARIES} 
    ${CGAL_3RD_PARTY_LIBRARIES}
    ${MONO_LIBRARIES}
    pthread
)

