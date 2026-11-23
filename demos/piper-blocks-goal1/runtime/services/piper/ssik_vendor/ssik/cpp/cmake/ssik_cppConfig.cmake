# Rendered for the ssik wheel by hatch_build.py (@PACKAGE_INIT@ equivalent).
macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()


# One config for both packagings of the headers: `cmake --install` of cpp/
# (CMakeLists.txt) and the ssik wheel (hatch_build.py, where
# ssik_cppWheelTargets.cmake provides ssik_cppTargets.cmake). Keep them in step.

# ssik_cpp is header-only. It links Eigen (header-only) and the platform thread
# library (parallel.hpp uses std::thread, #546), so the exported ssik::ssik_cpp
# target references Eigen3::Eigen and Threads::Threads -- a consumer needs both
# findable (brew install eigen / apt install libeigen3-dev; Threads is built in).
# No Eigen version is requested: Eigen's own version file would then reject
# another major (5.x on current Homebrew). ssik's gates and wheels use the release
# pinned in scripts/fetch_eigen.py (3.4.0); others can differ in the last bits.
include(CMakeFindDependencyMacro)
find_dependency(Eigen3 NO_MODULE)
find_dependency(Threads)

include("${CMAKE_CURRENT_LIST_DIR}/ssik_cppTargets.cmake")

check_required_components(ssik_cpp)
